#include "FirstOrderDynamics.hpp"

#include <csv.hpp>
#include <SQLiteCpp/SQLiteCpp.h>
#include <spdlog/spdlog.h>

static constexpr char CSV_SEPARATOR = ';';

namespace dsf::mobility {
  FirstOrderDynamics::FirstOrderDynamics(RoadNetwork&& graph,
                                         bool useCache,
                                         std::optional<unsigned int> seed)
      : Dynamics<RoadNetwork>(std::move(graph), seed), m_bCacheEnabled{useCache} {
    // Set defaults for speed function
    this->setSpeedFunction(SpeedFunction::LINEAR, 0.8);
    if (m_bCacheEnabled) {
      if (!std::filesystem::exists(CACHE_FOLDER)) {
        std::filesystem::create_directory(CACHE_FOLDER);
      }
      spdlog::info("Cache enabled (default folder is {})", CACHE_FOLDER);
    }
    for (auto const& [nodeId, pNode] : this->graph().nodes()) {
      m_nodeIndices.push_back(nodeId);
    }
    for (auto const& [nodeId, weight] : this->m_destinationNodes) {
      m_itineraries.emplace(nodeId, std::make_shared<Itinerary>(nodeId, nodeId));
    }
    std::for_each(
        this->graph().edges().cbegin(),
        this->graph().edges().cend(),
        [this](auto const& pair) {
          auto const& pEdge{pair.second};
          auto const edgeId{pair.first};
          // fill turn mapping as [pair.first, [left street Id, straight street Id, right street Id, U self street Id]]
          m_turnMapping.emplace(edgeId, std::array<long, 4>{-1, -1, -1, -1});
          // Turn mappings
          const auto& srcNodeId = pEdge->target();
          for (auto const& outEdgeId : this->graph().node(srcNodeId).outgoingEdges()) {
            auto* pStreet{&this->graph().edge(outEdgeId)};
            auto const previousStreetId = pStreet->id();
            auto const& delta{pEdge->deltaAngle(pStreet->angle())};
            if (std::abs(delta) < std::numbers::pi) {
              if (delta < 0.) {
                m_turnMapping[edgeId][dsf::Direction::RIGHT] = previousStreetId;  // right
              } else if (delta > 0.) {
                m_turnMapping[edgeId][dsf::Direction::LEFT] = previousStreetId;  // left
              } else {
                m_turnMapping[edgeId][dsf::Direction::STRAIGHT] =
                    previousStreetId;  // straight
              }
            } else {
              m_turnMapping[edgeId][dsf::Direction::UTURN] = previousStreetId;  // U
            }
          }
        });
  }

  std::unique_ptr<Agent> FirstOrderDynamics::m_killAgent(std::unique_ptr<Agent> pAgent) {
    spdlog::trace("Killing agent {}", *pAgent);
    m_travelDTs.push_back({pAgent->distance(),
                           static_cast<double>(this->time_step() - pAgent->spawnTime())});
    --m_nAgents;
    if (pAgent->isRandom() && !pAgent->hasArrived(this->time_step())) {
      ++m_nKilledAgents;
    }
    auto const& streetId = pAgent->streetId();
    if (streetId.has_value()) {
      auto* pStreet{&this->graph().edge(streetId.value())};
      auto const* pNode{&this->graph().node(pStreet->target())};
      auto [it, bInserted] = m_destinationCounts.insert({pNode->id(), 1});
      if (!bInserted) {
        ++it->second;
      }
    }
    return pAgent;
  }

  void FirstOrderDynamics::m_updatePath(std::shared_ptr<Itinerary> const& pItinerary) {
    if (m_bCacheEnabled) {
      auto const& file = std::format("{}{}.ity", CACHE_FOLDER, pItinerary->id());
      if (std::filesystem::exists(file)) {
        pItinerary->load(file);
        spdlog::debug("Loaded cached path for itinerary {}", pItinerary->id());
        return;
      }
    }
    auto const oldSize{pItinerary->path().size()};

    auto const path{this->graph().allPathsTo(pItinerary->destination())};
    pItinerary->setPath(path);
    auto const newSize{pItinerary->path().size()};
    if (oldSize > 0 && newSize != oldSize) {
      spdlog::debug("Path for itinerary {} changed size from {} to {}",
                    pItinerary->id(),
                    oldSize,
                    newSize);
    }
    if (m_bCacheEnabled) {
      pItinerary->save(std::format("{}{}.ity", CACHE_FOLDER, pItinerary->id()));
      spdlog::debug("Saved path in cache for itinerary {}", pItinerary->id());
    }
  }
  void FirstOrderDynamics::m_addAgentsRandom(std::size_t nAgents) {
    m_nAddedAgents += nAgents;
    std::uniform_real_distribution<double> uniformDist{0., 1.};
    std::exponential_distribution<double> distDist{1. /
                                                   m_meanTravelDistance.value_or(1.)};
    std::exponential_distribution<double> timeDist{1. / m_meanTravelTime.value_or(1.)};
    auto const bUniformSpawn{m_originNodes.empty()};
    if (m_originNodes.size() == 1) {
      auto [originId, weight] = m_originNodes.at(0);
      this->addAgents(nAgents, nullptr, originId);
      return;
    }
    while (nAgents--) {
      if (bUniformSpawn) {
        this->addAgent();
      } else {
        auto randValue{uniformDist(this->m_generator)};
        for (auto const& [origin, weight] : m_originNodes) {
          if (randValue < weight) {
            this->addAgent(nullptr, origin);
            break;
          }
          randValue -= weight;
        }
      }
      if (m_meanTravelDistance.has_value()) {
        auto const& pAgent{this->m_agents.back()};
        pAgent->setMaxDistance(distDist(this->m_generator));
      }
      if (m_meanTravelTime.has_value()) {
        auto const& pAgent{this->m_agents.back()};
        pAgent->setMaxTime(timeDist(this->m_generator));
      }
    }
  }
  void FirstOrderDynamics::m_addAgentsODs(std::size_t nAgents) {
    if (m_ODs.empty()) {
      throw std::runtime_error(
          "FirstOrderDynamics::m_addAgentsODs: No origin-destination pairs provided");
    }
    m_nAddedAgents += nAgents;
    if (m_timeToleranceFactor.has_value() && !m_agents.empty()) {
      auto const nStagnantAgents{m_agents.size()};
      spdlog::debug(
          "Removing {} stagnant agents that were not inserted since the previous call to "
          "addAgents(ODS).",
          nStagnantAgents);
      m_agents.clear();
      m_nAgents -= nStagnantAgents;
    }
    if (m_ODs.size() == 1) {
      auto [originId, destinationId, weight] = m_ODs.at(0);
      auto const itineraryIt = this->itineraries().find(destinationId);
      if (itineraryIt == this->itineraries().cend()) {
        spdlog::warn("Skipping ODS insertion: itinerary {} not found", destinationId);
        return;
      }
      this->addAgents(nAgents, itineraryIt->second, originId);
      return;
    }
    if (m_ODCumulativeWeights.size() != m_ODs.size()) {
      m_ODCumulativeWeights.clear();
      m_ODCumulativeWeights.reserve(m_ODs.size());
      double cumulativeWeight{0.};
      for (auto const& od : m_ODs) {
        cumulativeWeight += std::get<2>(od);
        m_ODCumulativeWeights.push_back(cumulativeWeight);
      }
      if (!m_ODCumulativeWeights.empty()) {
        m_ODCumulativeWeights.back() = 1.;
      }
    }
    std::uniform_real_distribution<double> uniformDist{0., m_ODCumulativeWeights.back()};
    while (nAgents--) {
      auto randValue = uniformDist(this->m_generator);
      auto selectedOdIdx = static_cast<std::size_t>(
          std::lower_bound(
              m_ODCumulativeWeights.cbegin(), m_ODCumulativeWeights.cend(), randValue) -
          m_ODCumulativeWeights.cbegin());
      if (selectedOdIdx >= m_ODs.size()) {
        selectedOdIdx = m_ODs.size() - 1;
      }

      auto const& originId = std::get<0>(m_ODs[selectedOdIdx]);
      auto const& destinationId = std::get<1>(m_ODs[selectedOdIdx]);
      auto const itineraryIt = this->itineraries().find(destinationId);
      if (itineraryIt == this->itineraries().cend()) {
        spdlog::warn("Skipping ODS insertion: itinerary {} not found", destinationId);
        continue;
      }
      this->addAgent(itineraryIt->second, originId);
    }
  }
  void FirstOrderDynamics::m_addAgentsRandomODs(std::size_t nAgents) {
    m_nAddedAgents += nAgents;
    if (m_timeToleranceFactor.has_value() && !m_agents.empty()) {
      auto const nStagnantAgents{m_agents.size()};
      spdlog::debug(
          "Removing {} stagnant agents that were not inserted since the previous call to "
          "addAgentsRandomly().",
          nStagnantAgents);
      m_agents.clear();
      m_nAgents -= nStagnantAgents;
    }
    auto const& nSources{m_originNodes.size()};
    auto const& nDestinations{m_destinationNodes.size()};
    spdlog::debug("Init addAgentsRandomly for {} agents from {} nodes to {} nodes.",
                  nAgents,
                  nSources,
                  nDestinations);
    if (nSources == 1 && nDestinations == 1 &&
        std::get<Id>(m_originNodes.at(0)) == std::get<Id>(m_destinationNodes.at(0))) {
      throw std::invalid_argument(
          std::format("The only source node {} is also the only destination node.",
                      std::get<Id>(m_originNodes.at(0))));
    }
    std::uniform_int_distribution<size_t> nodeDist{
        0, static_cast<size_t>(this->graph().nNodes() - 1)};
    std::uniform_real_distribution<double> uniformDist{0., 1.};
    spdlog::debug("Adding {} agents at time {}.", nAgents, this->time_step());
    while (nAgents--) {
      std::optional<Id> srcId{std::nullopt}, dstId{std::nullopt};

      // Select source using weighted random selection
      if (nSources == 1) {
        srcId = std::get<Id>(m_originNodes.at(0));
      } else {
        auto randValue = uniformDist(this->m_generator);
        for (const auto& [id, weight] : m_originNodes) {
          if (randValue < weight) {
            srcId = id;
            break;
          }
          randValue -= weight;
        }
      }

      // Select destination using weighted random selection
      if (nDestinations == 1) {
        dstId = std::get<Id>(m_destinationNodes.at(0));
      } else {
        auto randValue = uniformDist(this->m_generator);
        for (const auto& [id, weight] : m_destinationNodes) {
          if (randValue < weight) {
            dstId = id;
            break;
          }
          randValue -= weight;
        }
      }

      // Fallback to random nodes if selection failed
      if (!srcId.has_value()) {
        auto nodeIt{this->graph().nodes().begin()};
        std::advance(nodeIt, nodeDist(this->m_generator));
        srcId = nodeIt->first;
      }
      if (!dstId.has_value()) {
        auto nodeIt{this->graph().nodes().begin()};
        std::advance(nodeIt, nodeDist(this->m_generator));
        dstId = nodeIt->first;
      }

      // Find the itinerary with the given destination
      auto itineraryIt{std::find_if(this->itineraries().cbegin(),
                                    this->itineraries().cend(),
                                    [dstId](const auto& itinerary) {
                                      return itinerary.second->destination() == *dstId;
                                    })};
      if (itineraryIt == this->itineraries().cend()) {
        spdlog::error("Itinerary with destination {} not found. Skipping agent.", *dstId);
        continue;
      }

      // Check if destination is reachable from source
      auto const& itinerary = itineraryIt->second;
      if (!itinerary->path().contains(*srcId)) {
        spdlog::debug("Destination {} not reachable from source {}. Skipping agent.",
                      *dstId,
                      *srcId);
        continue;
      }

      this->addAgent(itineraryIt->second, *srcId);
    }
  }

  std::optional<Id> FirstOrderDynamics::m_nextStreetId(
      const std::unique_ptr<Agent>& pAgent, RoadJunction const* pNode) {
    spdlog::trace("Computing m_nextStreetId for {}", *pAgent);
    auto const& outgoingEdges = pNode->outgoingEdges();

    // Get current street information
    std::optional<Id> previousNodeId = std::nullopt;
    std::set<Id> forbiddenTurns;
    double speedCurrent{1.0};
    double lengthCurrent{1.0};
    if (pAgent->streetId().has_value()) {
      auto const* pStreetCurrent{&this->graph().edge(pAgent->streetId().value())};
      previousNodeId = pStreetCurrent->source();
      forbiddenTurns = pStreetCurrent->forbiddenTurns();
      speedCurrent = pStreetCurrent->maxSpeed();
      lengthCurrent = pStreetCurrent->length();
    }

    // Get path targets for non-random agents
    std::vector<Id> pathTargets;
    if (!pAgent->isRandom()) {
      auto const& path = pAgent->itinerary()->path();
      auto const pathIt = path.find(pNode->id());
      if (pathIt == path.cend()) {
        spdlog::debug(
            "No itinerary path entry for {} at node {}. Returning no transition.",
            *pAgent,
            pNode->id());
        return std::nullopt;
      }
      pathTargets = pathIt->second;
    }

    // Calculate transition probabilities for all valid outgoing edges
    std::unordered_map<Id, double> transitionProbabilities;
    double cumulativeProbability = 0.0;

    for (const auto outEdgeId : outgoingEdges) {
      auto const* pStreetOut{&this->graph().edge(outEdgeId)};

      // Check if this is a valid path target for non-random agents
      bool bIsPathTarget = false;
      if (!pathTargets.empty()) {
        bIsPathTarget =
            std::find(pathTargets.cbegin(), pathTargets.cend(), pStreetOut->target()) !=
            pathTargets.cend();
      }

      if (forbiddenTurns.contains(outEdgeId) && !bIsPathTarget) {
        continue;
      }

      if (!pathTargets.empty()) {
        if (!this->m_errorProbability.has_value() && !bIsPathTarget) {
          continue;
        }
      }

      // Calculate base probability
      auto const speedNext{pStreetOut->maxSpeed()};
      auto const lengthNext{pStreetOut->length()};
      double probability =
          std::sqrt((speedCurrent / lengthCurrent) * (speedNext / lengthNext));

      // Apply error probability for non-random agents
      if (this->m_errorProbability.has_value() && !pathTargets.empty()) {
        probability *=
            (bIsPathTarget
                 ? (1. - this->m_errorProbability.value())
                 : this->m_errorProbability.value() /
                       static_cast<double>(outgoingEdges.size() - pathTargets.size()));
      }

      // Handle U-turns
      if (previousNodeId.has_value() && pStreetOut->target() == previousNodeId.value()) {
        if (pNode->isRoundabout()) {
          probability *= U_TURN_PENALTY_FACTOR;
        } else if (!bIsPathTarget) {
          continue;  // No U-turns allowed
        }
      }

      transitionProbabilities[pStreetOut->id()] = probability;
      cumulativeProbability += probability;
    }

    // Select street based on weighted probabilities
    if (transitionProbabilities.empty()) {
      spdlog::debug("No valid transitions found for {} at {}", *pAgent, *pNode);
      return std::nullopt;
    }
    if (transitionProbabilities.size() == 1) {
      auto const& onlyStreetId = transitionProbabilities.cbegin()->first;
      spdlog::debug("Only one valid transition for {} at {}: street {}",
                    *pAgent,
                    *pNode,
                    onlyStreetId);
      return onlyStreetId;
    }

    std::uniform_real_distribution<double> uniformDist{0., cumulativeProbability};
    auto const randValue = uniformDist(this->m_generator);
    Id fallbackStreetId;
    double accumulated = 0.0;
    for (const auto& [targetStreetId, probability] : transitionProbabilities) {
      accumulated += probability;
      fallbackStreetId = targetStreetId;
      if (randValue < accumulated) {
        return targetStreetId;
      }
    }
    // Return last one as fallback
    spdlog::debug(
        "Fallback selection for {} at {}: street {}", *pAgent, *pNode, fallbackStreetId);
    return fallbackStreetId;
  }

  void FirstOrderDynamics::m_evolveStreet(Street* pStreet, bool reinsert_agents) {
    auto const nLanes = pStreet->nLanes();
    // Enqueue moving agents if their free time is up
    while (!pStreet->movingAgents().empty()) {
      auto const& pAgent{pStreet->movingAgents().top()};
      if (pAgent->freeTime() < this->time_step()) {
        break;
      }
      pAgent->setSpeed(0.);
      bool bArrived{false};
      if (!pAgent->isRandom()) {
        if (pAgent->itinerary()->destination() == pStreet->target()) {
          pAgent->updateItinerary();
        }
        if (pAgent->itinerary()->destination() == pStreet->target()) {
          bArrived = true;
        }
      }
      if (bArrived) {
        std::uniform_int_distribution<size_t> laneDist{0,
                                                       static_cast<size_t>(nLanes - 1)};
        pStreet->enqueue(laneDist(this->m_generator));
        continue;
      }
      auto const nextStreetId =
          this->m_nextStreetId(pAgent, &this->graph().node(pStreet->target()));
      if (!nextStreetId.has_value()) {
        spdlog::debug(
            "No next street found for agent {} at node {}", *pAgent, pStreet->target());
        if (pAgent->isRandom()) {
          std::uniform_int_distribution<size_t> laneDist{0,
                                                         static_cast<size_t>(nLanes - 1)};
          pStreet->enqueue(laneDist(this->m_generator));
          continue;
        }
        this->m_killAgent(pStreet->dequeueMovingAgent());
        continue;
        // Grufoony - 09/03/2026
        // The agent is now killed. The old behavior (throw exception) is kept here:
        //
        // throw std::runtime_error(std::format(
        //     "No next street found for agent {} at node {}", *pAgent, pStreet->target()));
      }
      auto const* pNextStreet{&this->graph().edge(nextStreetId.value())};
      pAgent->setNextStreetId(pNextStreet->id());
      if (nLanes == 1) {
        pStreet->enqueue(0);
        continue;
      }
      auto const direction{pNextStreet->turnDirection(pStreet->angle())};
      switch (direction) {
        case Direction::UTURN:
        case Direction::LEFT:
          pStreet->enqueue(nLanes - 1);
          break;
        case Direction::RIGHT:
          pStreet->enqueue(0);
          break;
        default:
          std::vector<double> weights;
          for (auto const& queue : pStreet->exitQueues()) {
            weights.push_back(1. / (queue.size() + 1));
          }
          // If all weights are the same, make the last 0
          if (std::all_of(weights.begin(), weights.end(), [&](double w) {
                return std::abs(w - weights.front()) <
                       std::numeric_limits<double>::epsilon();
              })) {
            weights.back() = 0.;
            if (nLanes > 2) {
              weights.front() = 0.;
            }
          }
          // Normalize the weights
          auto const sum = std::accumulate(weights.begin(), weights.end(), 0.);
          for (auto& w : weights) {
            w /= sum;
          }
          std::discrete_distribution<size_t> laneDist{weights.begin(), weights.end()};
          pStreet->enqueue(laneDist(this->m_generator));
      }
    }
    auto const& transportCapacity{pStreet->transportCapacity()};
    std::uniform_real_distribution<double> uniformDist{0., 1.};
    for (auto i = 0; i < std::ceil(transportCapacity); ++i) {
      if (i == std::ceil(transportCapacity) - 1) {
        double integral;
        double fractional = std::modf(transportCapacity, &integral);
        if (fractional != 0. && uniformDist(this->m_generator) > fractional) {
          spdlog::trace("Skipping due to fractional capacity {:.2f} < random value",
                        fractional);
          continue;
        }
      }
      for (auto queueIndex = 0; queueIndex < nLanes; ++queueIndex) {
        if (pStreet->queue(queueIndex).empty()) {
          continue;
        }
        // Logger::debug("Taking temp agent");
        auto const& pAgentTemp{pStreet->queue(queueIndex).front()};
        if (pAgentTemp->freeTime() > this->time_step()) {
          spdlog::trace("Skipping due to time {} < free time {}",
                        this->time_step(),
                        pAgentTemp->freeTime());
          continue;
        }

        if (m_timeToleranceFactor.has_value()) {
          auto const timeDiff{this->time_step() - pAgentTemp->freeTime()};
          auto const timeTolerance{m_timeToleranceFactor.value() *
                                   std::ceil(pStreet->length() / pStreet->maxSpeed())};
          if (timeDiff > timeTolerance) {
            spdlog::debug(
                "Time-step {} - {} currently on {} ({} turn - Traffic Light? {}), "
                "has been still for more than {} seconds ({} seconds). Killing it.",
                this->time_step(),
                *pAgentTemp,
                *pStreet,
                directionToString.at(pStreet->laneMapping().at(queueIndex)),
                this->graph().node(pStreet->target()).isTrafficLight(),
                timeTolerance,
                timeDiff);
            // Kill the agent
            this->m_killAgent(pStreet->dequeue(queueIndex, this->time_step()));
            continue;
          }
        }
        pAgentTemp->setSpeed(0.);
        auto* destinationNode{&this->graph().node(pStreet->target())};
        if (destinationNode->isFull()) {
          spdlog::trace("Skipping due to full destination node {}", *destinationNode);
          continue;
        }
        if (destinationNode->isTrafficLight()) {
          auto& tl = dynamic_cast<TrafficLight&>(*destinationNode);
          auto const direction{pStreet->laneMapping().at(queueIndex)};
          if (!tl.isGreen(pStreet->id(), direction)) {
            spdlog::trace("Skipping due to red light on street {} and direction {}",
                          pStreet->id(),
                          directionToString.at(direction));
            continue;
          }
          spdlog::debug("Green light on street {} and direction {}",
                        pStreet->id(),
                        directionToString.at(direction));
        } else if (destinationNode->isIntersection() &&
                   pAgentTemp->nextStreetId().has_value()) {
          auto& intersection = static_cast<Intersection&>(*destinationNode);
          bool bCanPass{true};
          if (!intersection.streetPriorities().empty()) {
            spdlog::debug("Checking priorities for street {} -> {}",
                          pStreet->source(),
                          pStreet->target());
            auto const& thisDirection{this->graph()
                                          .edge(pAgentTemp->nextStreetId().value())
                                          .turnDirection(pStreet->angle())};
            if (!intersection.streetPriorities().contains(pStreet->id())) {
              // I have to check if the agent has right of way
              auto const& inNeighbours{destinationNode->ingoingEdges()};
              for (auto const& inEdgeId : inNeighbours) {
                auto const* pStreetTemp{&this->graph().edge(inEdgeId)};
                if (pStreetTemp->id() == pStreet->id()) {
                  continue;
                }
                if (pStreetTemp->nExitingAgents() == 0) {
                  continue;
                }
                if (intersection.streetPriorities().contains(pStreetTemp->id())) {
                  spdlog::debug(
                      "Skipping agent emission from street {} -> {} due to right of way.",
                      pStreet->source(),
                      pStreet->target());
                  bCanPass = false;
                  break;
                } else if (thisDirection >= Direction::LEFT) {
                  // Check if the agent has right of way using direction
                  // The problem arises only when you have to turn left
                  for (auto i{0}; i < pStreetTemp->nLanes(); ++i) {
                    // check queue is not empty and take the top agent
                    if (pStreetTemp->queue(i).empty()) {
                      continue;
                    }
                    auto const& pAgentTemp2{pStreetTemp->queue(i).front()};
                    if (!pAgentTemp2->nextStreetId().has_value()) {
                      continue;
                    }
                    auto const& otherDirection{
                        this->graph()
                            .edge(pAgentTemp2->nextStreetId().value())
                            .turnDirection(this->graph()
                                               .edge(pAgentTemp2->streetId().value())
                                               .angle())};
                    if (otherDirection < Direction::LEFT) {
                      spdlog::debug(
                          "Skipping agent emission from street {} -> {} due to right of "
                          "way with other agents.",
                          pStreet->source(),
                          pStreet->target());
                      bCanPass = false;
                      break;
                    }
                  }
                }
              }
            } else if (thisDirection >= Direction::LEFT) {
              for (auto const& streetId : intersection.streetPriorities()) {
                if (streetId == pStreet->id()) {
                  continue;
                }
                auto const* pStreetTemp{&this->graph().edge(streetId)};
                for (auto i{0}; i < pStreetTemp->nLanes(); ++i) {
                  // check queue is not empty and take the top agent
                  if (pStreetTemp->queue(i).empty()) {
                    continue;
                  }
                  auto const& pAgentTemp2{pStreetTemp->queue(i).front()};
                  if (!pAgentTemp2->streetId().has_value()) {
                    continue;
                  }
                  auto const& otherDirection{
                      this->graph()
                          .edge(pAgentTemp2->nextStreetId().value())
                          .turnDirection(this->graph()
                                             .edge(pAgentTemp2->streetId().value())
                                             .angle())};
                  if (otherDirection < thisDirection) {
                    spdlog::debug(
                        "Skipping agent emission from street {} -> {} due to right of "
                        "way with other agents.",
                        pStreet->source(),
                        pStreet->target());
                    bCanPass = false;
                    break;
                  }
                }
              }
            }
          }
          if (!bCanPass) {
            spdlog::debug(
                "Skipping agent emission from street {} -> {} due to right of way",
                pStreet->source(),
                pStreet->target());
            continue;
          }
        }
        bool bArrived{false};
        if (!(uniformDist(this->m_generator) <
              m_passageProbability.value_or(std::numeric_limits<double>::max()))) {
          if (pAgentTemp->isRandom()) {
            bArrived = true;
          } else {
            spdlog::debug(
                "Skipping agent emission from street {} -> {} due to passage "
                "probability",
                pStreet->source(),
                pStreet->target());
            continue;
          }
        }
        if (!pAgentTemp->isRandom()) {
          if (destinationNode->id() == pAgentTemp->itinerary()->destination()) {
            bArrived = true;
            spdlog::debug("Agent {} has arrived at destination node {}",
                          pAgentTemp->id(),
                          destinationNode->id());
          }
        } else {
          if (!pAgentTemp->nextStreetId().has_value()) {
            bArrived = true;
            spdlog::debug("Random agent {} has arrived at destination node {}",
                          pAgentTemp->id(),
                          destinationNode->id());
          } else if (pAgentTemp->hasArrived(this->time_step())) {
            bArrived = true;
          }
        }
        if (bArrived) {
          auto pAgent =
              this->m_killAgent(pStreet->dequeue(queueIndex, this->time_step()));
          ++m_nArrivedAgents;
          if (reinsert_agents) {
            // reset Agent's values
            pAgent->reset(this->time_step());
            this->addAgent(std::move(pAgent));
          }
          continue;
        }
        if (!pAgentTemp->streetId().has_value()) {
          spdlog::error("{} has no street id", *pAgentTemp);
        }
        auto* nextStreet{&this->graph().edge(pAgentTemp->nextStreetId().value())};
        if (nextStreet->isFull()) {
          spdlog::debug(
              "Skipping agent emission from street {} -> {} due to full "
              "next street: {}",
              pStreet->source(),
              pStreet->target(),
              *nextStreet);
          continue;
        }
        auto pAgent{pStreet->dequeue(queueIndex, this->time_step())};
        spdlog::debug(
            "{} at time {} has been dequeued from street {} and enqueued on street {} "
            "with free time {}.",
            *pAgent,
            this->time_step(),
            pStreet->id(),
            nextStreet->id(),
            pAgent->freeTime());
        assert(destinationNode->id() == nextStreet->source());
        if (destinationNode->isIntersection()) {
          auto& intersection = dynamic_cast<Intersection&>(*destinationNode);
          auto const delta{nextStreet->deltaAngle(pStreet->angle())};
          intersection.addAgent(delta, std::move(pAgent));
        } else if (destinationNode->isRoundabout()) {
          auto& roundabout = dynamic_cast<Roundabout&>(*destinationNode);
          roundabout.enqueue(std::move(pAgent));
        }
      }
    }
  }

  void FirstOrderDynamics::m_evolveNode(RoadJunction* pNode) {
    auto const transportCapacity{pNode->transportCapacity()};
    for (auto i{0}; i < std::ceil(transportCapacity); ++i) {
      if (i == std::ceil(transportCapacity) - 1) {
        std::uniform_real_distribution<double> uniformDist{0., 1.};
        double integral;
        double fractional = std::modf(transportCapacity, &integral);
        if (fractional != 0. && uniformDist(this->m_generator) > fractional) {
          spdlog::debug("Skipping dequeue from node {} due to transport capacity {}",
                        pNode->id(),
                        transportCapacity);
          return;
        }
      }
      if (pNode->isIntersection()) {
        auto& intersection = dynamic_cast<Intersection&>(*pNode);
        if (intersection.agents().empty()) {
          return;
        }
        for (auto it{intersection.agents().begin()}; it != intersection.agents().end();) {
          auto& pAgent{it->second};
          auto* nextStreet{&this->graph().edge(pAgent->nextStreetId().value())};
          if (nextStreet->isFull()) {
            spdlog::debug("Next street is full: {}", *nextStreet);
            if (m_forcePriorities) {
              spdlog::debug("Forcing priority from {} on {}", *pNode, *nextStreet);
              return;
            }
            ++it;
            continue;
          }
          if (!m_turnCounts.empty() && pAgent->streetId().has_value()) {
            ++m_turnCounts[*(pAgent->streetId())][nextStreet->id()];
          }
          pAgent->setStreetId();
          pAgent->setSpeed(this->m_speedFunction(*nextStreet));
          pAgent->setFreeTime(this->time_step() +
                              std::ceil(nextStreet->length() / pAgent->speed()));
          spdlog::debug(
              "{} at time {} has been dequeued from intersection {} and "
              "enqueued on street {} with free time {}.",
              *pAgent,
              this->time_step(),
              pNode->id(),
              nextStreet->id(),
              pAgent->freeTime());
          nextStreet->addAgent(std::move(pAgent), this->time_step());
          it = intersection.agents().erase(it);
          break;
        }
      } else if (pNode->isRoundabout()) {
        auto& roundabout = dynamic_cast<Roundabout&>(*pNode);
        if (roundabout.agents().empty()) {
          return;
        }
        auto const& pAgentTemp{roundabout.agents().front()};
        auto* nextStreet{&this->graph().edge(pAgentTemp->nextStreetId().value())};
        if (!(nextStreet->isFull())) {
          if (!m_turnCounts.empty() && pAgentTemp->streetId().has_value()) {
            ++m_turnCounts[*(pAgentTemp->streetId())][nextStreet->id()];
          }
          auto pAgent{roundabout.dequeue()};
          pAgent->setStreetId();
          pAgent->setSpeed(this->m_speedFunction(*nextStreet));
          pAgent->setFreeTime(this->time_step() +
                              std::ceil(nextStreet->length() / pAgent->speed()));
          spdlog::debug(
              "An agent at time {} has been dequeued from roundabout {} and "
              "enqueued on street {} with free time {}: {}",
              this->time_step(),
              pNode->id(),
              nextStreet->id(),
              pAgent->freeTime(),
              *pAgent);
          nextStreet->addAgent(std::move(pAgent), this->time_step());
        } else {
          return;
        }
      }
    }
  }
  void FirstOrderDynamics::m_evolveAgents() {
    if (m_agents.empty()) {
      spdlog::trace("No agents to process.");
      return;
    }
    std::uniform_int_distribution<Id> nodeDist{
        0, static_cast<Id>(this->graph().nNodes() - 1)};
    spdlog::debug("Processing {} agents", m_agents.size());
    for (auto itAgent{m_agents.begin()}; itAgent != m_agents.end();) {
      auto& pAgent{*itAgent};
      if (!pAgent->srcNodeId().has_value()) {
        auto nodeIt{this->graph().nodes().begin()};
        std::advance(nodeIt, nodeDist(this->m_generator));
        pAgent->setSrcNodeId(nodeIt->second->id());
      }
      auto* pSourceNode{&this->graph().node(*(pAgent->srcNodeId()))};
      if (pSourceNode->isFull()) {
        spdlog::debug("Skipping {} due to full source {}", *pAgent, *pSourceNode);
        ++itAgent;
        continue;
      }
      if (!pAgent->nextStreetId().has_value()) {
        spdlog::debug("No next street id, generating a random one");
        auto const nextStreetId{this->m_nextStreetId(pAgent, pSourceNode)};
        if (!nextStreetId.has_value()) {
          spdlog::debug(
              "No next street found for agent {} at node {}", *pAgent, pSourceNode->id());
          itAgent = m_agents.erase(itAgent);
          continue;
        }
        pAgent->setNextStreetId(nextStreetId.value());
      }
      // spdlog::debug("Checking next street {}", pAgent->nextStreetId().value());
      auto* nextStreet{&this->graph().edge(pAgent->nextStreetId().value())};
      if (nextStreet->isFull()) {
        ++itAgent;
        spdlog::debug("Skipping {} due to full input {}", *pAgent, *nextStreet);
        continue;
      }
      // spdlog::debug("Adding agent on the source node");
      if (pSourceNode->isIntersection()) {
        auto& intersection = dynamic_cast<Intersection&>(*pSourceNode);
        intersection.addAgent(0., std::move(pAgent));
      } else if (pSourceNode->isRoundabout()) {
        auto& roundabout = dynamic_cast<Roundabout&>(*pSourceNode);
        roundabout.enqueue(std::move(pAgent));
      }
      itAgent = m_agents.erase(itAgent);
    }
    spdlog::debug("There are {} agents left in the list.", m_agents.size());
  }

  // Init Street Data methods
  void FirstOrderDynamics::m_initStreetTable() const {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    // Create table if it doesn't exist
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS road_data ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "simulation_id INTEGER NOT NULL, "
        "datetime TEXT NOT NULL, "
        "time_step INTEGER NOT NULL, "
        "street_id INTEGER NOT NULL, "
        "coil TEXT, "
        "density_vpk REAL, "
        "avg_speed_kph REAL, "
        "std_speed_kph REAL, "
        "n_observations INTEGER, "
        "counts INTEGER, "
        "queue_length INTEGER)");

    spdlog::info("Initialized road_data table in the database.");
  }
  void FirstOrderDynamics::m_saveStreetDataSQL(
      const std::string& datetime,
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_map<Id, StreetDataRecord> streetDataRecords) const {
    if (streetDataRecords.empty()) {
      spdlog::debug("No street data records to save for time step {}.", time_step);
      return;
    }
    SQLite::Statement insertStmt(
        *this->database(),
        "INSERT INTO road_data (datetime, time_step, simulation_id, street_id, "
        "coil, density_vpk, avg_speed_kph, std_speed_kph, n_observations, counts, "
        "queue_length) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");

    for (auto const& [streetId, record] : streetDataRecords) {
      insertStmt.bind(1, datetime);
      insertStmt.bind(2, time_step);
      insertStmt.bind(3, simulation_id);
      insertStmt.bind(4, static_cast<std::int64_t>(streetId));
      if (record.coilName.has_value()) {
        insertStmt.bind(5, record.coilName.value());
      } else {
        insertStmt.bind(5);
      }
      insertStmt.bind(6, record.density);
      if (record.avgSpeed.has_value()) {
        insertStmt.bind(7, record.avgSpeed.value());
        insertStmt.bind(8, record.stdSpeed.value());
      } else {
        insertStmt.bind(7);
        insertStmt.bind(8);
      }
      insertStmt.bind(9, static_cast<std::int64_t>(record.nObservations.value_or(0)));
      if (record.counts.has_value()) {
        insertStmt.bind(10, static_cast<std::int64_t>(record.counts.value()));
      } else {
        insertStmt.bind(10);
      }
      insertStmt.bind(11, static_cast<std::int64_t>(record.queueLength));
      insertStmt.exec();
      insertStmt.reset();
    }
  }
  void FirstOrderDynamics::m_saveStreetDataCSV(
      const std::string& datetime,
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_map<Id, StreetDataRecord> streetDataRecords) const {
    if (streetDataRecords.empty()) {
      spdlog::debug("No street data records to save for time step {}.", time_step);
      return;
    }
    auto const filename{
        std::format("{}_{}_road_data.csv", simulation_id, this->m_safeName())};
    // Check if the file exists to write the header
    bool fileExists = std::filesystem::exists(filename);
    std::ofstream outFile(filename, std::ios::app);
    if (!outFile.is_open()) {
      spdlog::error("Failed to open file {} for writing street data.", filename);
      return;
    }
    // Header
    if (!fileExists) {
      outFile << "datetime" << CSV_SEPARATOR << "time_step" << CSV_SEPARATOR
              << "street_id" << CSV_SEPARATOR << "coil" << CSV_SEPARATOR << "density_vpk"
              << CSV_SEPARATOR << "avg_speed_kph" << CSV_SEPARATOR << "std_speed_kph"
              << CSV_SEPARATOR << "n_observations" << CSV_SEPARATOR << "counts"
              << CSV_SEPARATOR << "queue_length" << "\n";
    }
    // Data rows
    for (auto const& [streetId, record] : streetDataRecords) {
      outFile << datetime << CSV_SEPARATOR << time_step << CSV_SEPARATOR << streetId
              << CSV_SEPARATOR;
      if (record.coilName.has_value()) {
        outFile << record.coilName.value();
      }
      outFile << CSV_SEPARATOR << record.density << CSV_SEPARATOR;
      if (record.avgSpeed.has_value()) {
        outFile << record.avgSpeed.value() << CSV_SEPARATOR << record.stdSpeed.value()
                << CSV_SEPARATOR;
      } else {
        outFile << CSV_SEPARATOR << CSV_SEPARATOR;
      }
      outFile << record.nObservations.value_or(0) << CSV_SEPARATOR;
      if (record.counts.has_value()) {
        outFile << record.counts.value();
      }
      outFile << CSV_SEPARATOR << record.queueLength << "\n";
    }
    // Flush and close the file
    outFile.flush();
    outFile.close();
    spdlog::debug("Saved street data for time step {} to file {}.", time_step, filename);
  }
  // End Street Data methods
  // Init Avg Stats methods
  void FirstOrderDynamics::m_initAvgStatsTable() const {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    // Create table if it doesn't exist
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS avg_stats ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "simulation_id INTEGER NOT NULL, "
        "datetime TEXT NOT NULL, "
        "time_step INTEGER NOT NULL, "
        "n_ghost_agents INTEGER NOT NULL, "
        "n_agents INTEGER NOT NULL, "
        "mean_speed_kph REAL, "
        "std_speed_kph REAL, "
        "mean_density_vpk REAL NOT NULL, "
        "std_density_vpk REAL NOT NULL, "
        "mean_travel_time_s REAL, "
        "mean_queue_length REAL NOT NULL)");

    spdlog::info("Initialized avg_stats table in the database.");
  }
  void FirstOrderDynamics::m_saveAvgStatsSQL(const std::string& datetime,
                                             const std::int64_t time_step,
                                             const std::int64_t simulation_id,
                                             const std::size_t n_valid_edges,
                                             const double mean_speed,
                                             const double std_speed,
                                             const double mean_density,
                                             const double std_density,
                                             const double mean_traveltime,
                                             const double meanQueueLength) const {
    SQLite::Statement insertStmt(
        *this->database(),
        "INSERT INTO avg_stats ("
        "simulation_id, datetime, time_step, n_ghost_agents, n_agents, "
        "mean_speed_kph, std_speed_kph, mean_density_vpk, std_density_vpk, "
        "mean_travel_time_s, mean_queue_length) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
    insertStmt.bind(1, simulation_id);
    insertStmt.bind(2, datetime);
    insertStmt.bind(3, time_step);
    insertStmt.bind(4, static_cast<std::int64_t>(m_agents.size()));
    insertStmt.bind(5, static_cast<std::int64_t>(this->nAgents()));

    if (n_valid_edges > 0) {
      insertStmt.bind(6, mean_speed);
      insertStmt.bind(7, std_speed);
      insertStmt.bind(10, mean_traveltime);
    } else {
      insertStmt.bind(6);
      insertStmt.bind(7);
      insertStmt.bind(10);
    }
    insertStmt.bind(8, mean_density);
    insertStmt.bind(9, std_density);
    insertStmt.bind(11, meanQueueLength);
    insertStmt.exec();
  }
  void FirstOrderDynamics::m_saveAvgStatsCSV(const std::string& datetime,
                                             const std::int64_t time_step,
                                             const std::int64_t simulation_id,
                                             const std::size_t n_valid_edges,
                                             const double mean_speed,
                                             const double std_speed,
                                             const double mean_density,
                                             const double std_density,
                                             const double mean_traveltime,
                                             const double meanQueueLength) const {
    auto const filename{
        std::format("{}_{}_avg_stats.csv", simulation_id, this->m_safeName())};
    // Check if the file exists to write the header
    bool fileExists = std::filesystem::exists(filename);
    std::ofstream outFile(filename, std::ios::app);
    if (!outFile.is_open()) {
      spdlog::error("Failed to open file {} for writing average stats.", filename);
      return;
    }
    // Header
    if (!fileExists) {
      outFile << "datetime" << CSV_SEPARATOR << "time_step" << CSV_SEPARATOR
              << "n_ghost_agents" << CSV_SEPARATOR << "n_agents" << CSV_SEPARATOR
              << "mean_speed_kph" << CSV_SEPARATOR << "std_speed_kph" << CSV_SEPARATOR
              << "mean_density_vpk" << CSV_SEPARATOR << "std_density_vpk" << CSV_SEPARATOR
              << "mean_travel_time_s" << CSV_SEPARATOR << "mean_queue_length" << "\n";
    }
    // Data row
    outFile << datetime << CSV_SEPARATOR << time_step << CSV_SEPARATOR << m_agents.size()
            << CSV_SEPARATOR << this->nAgents() << CSV_SEPARATOR;
    if (n_valid_edges > 0) {
      outFile << mean_speed << CSV_SEPARATOR << std_speed << CSV_SEPARATOR << mean_density
              << CSV_SEPARATOR << std_density << CSV_SEPARATOR << mean_traveltime
              << CSV_SEPARATOR;
    } else {
      outFile << CSV_SEPARATOR << CSV_SEPARATOR << mean_density << CSV_SEPARATOR
              << std_density << CSV_SEPARATOR << CSV_SEPARATOR;
    }
    outFile << meanQueueLength << "\n";
    // Flush and close the file
    outFile.flush();
    outFile.close();
    spdlog::debug(
        "Saved average stats for time step {} to file {}.", time_step, filename);
  }
  // End Avg Stats methods
  // Init Travel Data methods
  void FirstOrderDynamics::m_initTravelDataTable() const {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    // Create table if it doesn't exist
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS travel_data ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "simulation_id INTEGER NOT NULL, "
        "datetime TEXT NOT NULL, "
        "time_step INTEGER NOT NULL, "
        "distance_m REAL NOT NULL, "
        "travel_time_s REAL NOT NULL)");

    spdlog::info("Initialized travel_data table in the database.");
  }
  void FirstOrderDynamics::m_saveTravelDataSQL(
      const std::string& datetime,
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_vector<std::pair<double, double>> travelDTs) const {
    SQLite::Statement insertStmt(*this->database(),
                                 "INSERT INTO travel_data (datetime, time_step, "
                                 "simulation_id, distance_m, travel_time_s) "
                                 "VALUES (?, ?, ?, ?, ?)");

    for (auto const& [distance, time] : travelDTs) {
      insertStmt.bind(1, datetime);
      insertStmt.bind(2, time_step);
      insertStmt.bind(3, simulation_id);
      insertStmt.bind(4, distance);
      insertStmt.bind(5, time);
      insertStmt.exec();
      insertStmt.reset();
    }
  }
  void FirstOrderDynamics::m_saveTravelDataCSV(
      const std::string& datetime,
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_vector<std::pair<double, double>> travelDTs) const {
    auto const filename{
        std::format("{}_{}_travel_data.csv", simulation_id, this->m_safeName())};
    // Check if the file exists to write the header
    bool fileExists = std::filesystem::exists(filename);
    std::ofstream outFile(filename, std::ios::app);
    if (!outFile.is_open()) {
      spdlog::error("Failed to open file {} for writing travel data.", filename);
      return;
    }
    // Header
    if (!fileExists) {
      outFile << "datetime" << CSV_SEPARATOR << "time_step" << CSV_SEPARATOR
              << "distance_m" << CSV_SEPARATOR << "travel_time_s" << "\n";
    }
    // Data rows
    for (auto const& [distance, time] : travelDTs) {
      outFile << datetime << CSV_SEPARATOR << time_step << CSV_SEPARATOR << distance
              << CSV_SEPARATOR << time << "\n";
    }
    // Flush and close the file
    outFile.flush();
    outFile.close();
    spdlog::debug("Saved travel data for time step {} to file {}.", time_step, filename);
  }
  // End Travel Data methods
  // Init Agent Data methods
  void FirstOrderDynamics::m_initAgentDataTable() const {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    // Create table if it doesn't exist
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS agent_data ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "simulation_id INTEGER NOT NULL, "
        "agent_id INTEGER NOT NULL, "
        "edge_id INTEGER NOT NULL, "
        "time_step_in INTEGER NOT NULL, "
        "time_step_out INTEGER NOT NULL)");

    spdlog::info("Initialized agent_data table in the database.");
  }
  void FirstOrderDynamics::m_saveAgentDataSQL(
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_unordered_map<Id,
                                    std::vector<std::tuple<Id, std::time_t, std::time_t>>>
          agentDataRecords) const {
    if (agentDataRecords.empty()) {
      spdlog::debug("No agent data records to save for time step {}.", time_step);
      return;
    }
    SQLite::Statement insertStmt(*this->database(),
                                 "INSERT INTO agent_data (simulation_id, "
                                 "agent_id, edge_id, time_step_in, time_step_out)"
                                 "VALUES (?, ?, ?, ?, ?)");
    for (auto const& [edge_id, data] : agentDataRecords) {
      for (auto const& [agent_id, ts_in, ts_out] : data) {
        insertStmt.bind(1, simulation_id);
        insertStmt.bind(2, static_cast<std::int64_t>(agent_id));
        insertStmt.bind(3, static_cast<std::int64_t>(edge_id));
        insertStmt.bind(4, static_cast<std::int64_t>(ts_in));
        insertStmt.bind(5, static_cast<std::int64_t>(ts_out));
        insertStmt.exec();
        insertStmt.reset();
      }
    }
  }
  void FirstOrderDynamics::m_saveAgentDataCSV(
      const std::int64_t time_step,
      const std::int64_t simulation_id,
      tbb::concurrent_unordered_map<Id,
                                    std::vector<std::tuple<Id, std::time_t, std::time_t>>>
          agentDataRecords) const {
    if (agentDataRecords.empty()) {
      spdlog::debug("No agent data records to save for time step {}.", time_step);
      return;
    }
    auto const filename{
        std::format("{}_{}_agent_data.csv", simulation_id, this->m_safeName())};
    // Check if the file exists to write the header
    bool fileExists = std::filesystem::exists(filename);
    std::ofstream outFile(filename, std::ios::app);
    if (!outFile.is_open()) {
      spdlog::error("Failed to open file {} for writing agent data.", filename);
      return;
    }
    // Header
    if (!fileExists) {
      outFile << "simulation_id" << CSV_SEPARATOR << "agent_id" << CSV_SEPARATOR
              << "edge_id" << CSV_SEPARATOR << "time_step_in" << CSV_SEPARATOR
              << "time_step_out" << "\n";
    }
    // Data rows
    for (auto const& [edge_id, data] : agentDataRecords) {
      for (auto const& [agent_id, ts_in, ts_out] : data) {
        outFile << simulation_id << CSV_SEPARATOR << agent_id << CSV_SEPARATOR << edge_id
                << CSV_SEPARATOR << ts_in << CSV_SEPARATOR << ts_out << "\n";
      }
    }
    // Flush and close the file
    outFile.flush();
    outFile.close();
    spdlog::debug("Saved agent data for time step {} to file {}.", time_step, filename);
  }
  // End Agent Data methods
  void FirstOrderDynamics::m_dumpSimInfo() const {
    // Dump simulation info (parameters) to the database, if connected
    if (!this->database()) {
      return;
    }
    // Create simulations table if it doesn't exist
    SQLite::Statement createTableStmt(*this->database(),
                                      "CREATE TABLE IF NOT EXISTS simulations ("
                                      "id INTEGER PRIMARY KEY, "
                                      "name TEXT, "
                                      "speed_function TEXT, "
                                      "error_probability REAL, "
                                      "passage_probability REAL, "
                                      "mean_travel_distance_m REAL, "
                                      "mean_travel_time_s REAL, "
                                      "stagnant_tolerance_factor REAL, "
                                      "force_priorities BOOLEAN, "
                                      "save_avg_stats BOOLEAN, "
                                      "save_road_data BOOLEAN, "
                                      "save_travel_data BOOLEAN, "
                                      "save_agent_data BOOLEAN)");
    createTableStmt.exec();
    // Insert simulation parameters into the simulations table
    SQLite::Statement insertSimStmt(
        *this->database(),
        "INSERT INTO simulations (id, name, speed_function, error_probability, "
        "passage_probability, mean_travel_distance_m, mean_travel_time_s, "
        "stagnant_tolerance_factor, force_priorities, save_avg_stats, save_road_data, "
        "save_travel_data, save_agent_data) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");
    insertSimStmt.bind(1, static_cast<std::int64_t>(this->id()));
    insertSimStmt.bind(2, this->name());
    insertSimStmt.bind(3, this->m_speedFunctionDescription);
    if (this->m_errorProbability.has_value()) {
      insertSimStmt.bind(4, *this->m_errorProbability);
    } else {
      insertSimStmt.bind(4);
    }
    if (this->m_passageProbability.has_value()) {
      insertSimStmt.bind(5, *this->m_passageProbability);
    } else {
      insertSimStmt.bind(5);
    }
    if (this->m_meanTravelDistance.has_value()) {
      insertSimStmt.bind(6, *this->m_meanTravelDistance);
    } else {
      insertSimStmt.bind(6);
    }
    if (this->m_meanTravelTime.has_value()) {
      insertSimStmt.bind(7, static_cast<int64_t>(*this->m_meanTravelTime));
    } else {
      insertSimStmt.bind(7);
    }
    if (this->m_timeToleranceFactor.has_value()) {
      insertSimStmt.bind(8, *this->m_timeToleranceFactor);
    } else {
      insertSimStmt.bind(8);
    }
    insertSimStmt.bind(9, this->m_forcePriorities);
    insertSimStmt.bind(10, this->m_bSaveAverageStats);
    insertSimStmt.bind(11, this->m_bSaveStreetData);
    insertSimStmt.bind(12, this->m_bSaveTravelData);
    insertSimStmt.bind(13, this->m_bSaveAgentData);
    insertSimStmt.exec();
  }
  void FirstOrderDynamics::m_dumpNetwork() const {
    if (!this->database()) {
      throw std::runtime_error(
          "No database connected. Call connectDataBase() before saving data.");
    }
    // Check if edges and nodes tables already exists
    SQLite::Statement edgesQuery(
        *this->database(),
        "SELECT name FROM sqlite_master WHERE type='table' AND name='edges';");
    SQLite::Statement nodesQuery(
        *this->database(),
        "SELECT name FROM sqlite_master WHERE type='table' AND name='nodes';");
    bool edgesTableExists = edgesQuery.executeStep();
    bool nodesTableExists = nodesQuery.executeStep();
    if (edgesTableExists && nodesTableExists) {
      spdlog::info(
          "Edges and nodes tables already exist in the database. Skipping network dump.");
      return;
    }

    // Create edges table
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS edges ("
        "id INTEGER PRIMARY KEY, "
        "source INTEGER NOT NULL, "
        "target INTEGER NOT NULL, "
        "length REAL NOT NULL, "
        "maxspeed REAL NOT NULL, "
        "name TEXT, "
        "nlanes INTEGER NOT NULL, "
        "coilcode TEXT, "
        "geometry TEXT NOT NULL)");
    // Create nodes table
    this->database()->exec(
        "CREATE TABLE IF NOT EXISTS nodes ("
        "id INTEGER PRIMARY KEY, "
        "type TEXT, "
        "geometry TEXT)");

    // Insert edges
    SQLite::Statement insertEdgeStmt(*this->database(),
                                     "INSERT INTO edges (id, source, target, length, "
                                     "maxspeed, name, nlanes, coilcode, geometry) "
                                     "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?);");
    for (const auto& [edgeId, pEdge] : this->graph().edges()) {
      insertEdgeStmt.bind(1, static_cast<std::int64_t>(edgeId));
      insertEdgeStmt.bind(2, static_cast<std::int64_t>(pEdge->source()));
      insertEdgeStmt.bind(3, static_cast<std::int64_t>(pEdge->target()));
      insertEdgeStmt.bind(4, pEdge->length());
      insertEdgeStmt.bind(5, pEdge->maxSpeed());
      insertEdgeStmt.bind(6, pEdge->name());
      insertEdgeStmt.bind(7, pEdge->nLanes());
      auto const& counterName{pEdge->counterName()};
      if (counterName != "N/A") {
        insertEdgeStmt.bind(8, counterName);
      } else {
        insertEdgeStmt.bind(8);
      }
      insertEdgeStmt.bind(9, std::format("{}", pEdge->geometry()));
      insertEdgeStmt.exec();
      insertEdgeStmt.reset();
    }
    // Insert nodes
    SQLite::Statement insertNodeStmt(
        *this->database(), "INSERT INTO nodes (id, type, geometry) VALUES (?, ?, ?);");
    for (const auto& [nodeId, pNode] : this->graph().nodes()) {
      insertNodeStmt.bind(1, static_cast<std::int64_t>(nodeId));
      if (pNode->isTrafficLight()) {
        insertNodeStmt.bind(2, "traffic_light");
      } else if (pNode->isRoundabout()) {
        insertNodeStmt.bind(2, "roundabout");
      } else {
        insertNodeStmt.bind(2);
      }
      if (pNode->geometry().has_value()) {
        insertNodeStmt.bind(3, std::format("{}", *pNode->geometry()));
      } else {
        insertNodeStmt.bind(3);
      }
      insertNodeStmt.exec();
      insertNodeStmt.reset();
    }
  }

  void FirstOrderDynamics::setErrorProbability(double errorProbability) {
    if (errorProbability < 0. || errorProbability > 1.) {
      throw std::invalid_argument(
          std::format("The error probability ({}) must be in [0, 1]", errorProbability));
    }
    m_errorProbability = errorProbability;
  }
  void FirstOrderDynamics::setPassageProbability(double passageProbability) {
    if (passageProbability < 0. || passageProbability > 1.) {
      throw std::invalid_argument(std::format(
          "The passage probability ({}) must be in [0, 1]", passageProbability));
    }
    m_passageProbability = passageProbability;
  }
  void FirstOrderDynamics::killStagnantAgents(double timeToleranceFactor) {
    if (timeToleranceFactor <= 0.) {
      throw std::invalid_argument(std::format(
          "The time tolerance factor ({}) must be positive", timeToleranceFactor));
    }
    m_timeToleranceFactor = timeToleranceFactor;
  }
  void FirstOrderDynamics::setOriginNodes(
      std::unordered_map<Id, double> const& originNodes) {
    m_originNodes.clear();
    m_originNodes.reserve(originNodes.size());
    if (originNodes.empty()) {
      // If no origin nodes are provided, try to set origin nodes basing on streets' stationary weights
      auto const nEdges{this->graph().nEdges()};
      for (auto const& [edgeId, pEdge] : this->graph().edges()) {
        m_originNodes.push_back({pEdge->source(), 1. / nEdges});
      }
      return;
    }
    auto const sumWeights = std::accumulate(
        originNodes.begin(), originNodes.end(), 0., [](double sum, auto const& pair) {
          return sum + pair.second;
        });
    if (sumWeights <= 0.) {
      throw std::invalid_argument(
          std::format("The sum of the weights ({}) must be positive", sumWeights));
    }
    if (sumWeights == 1.) {
      std::copy(
          originNodes.begin(), originNodes.end(), std::back_inserter(m_originNodes));
      return;
    }
    std::transform(originNodes.begin(),
                   originNodes.end(),
                   std::back_inserter(m_originNodes),
                   [sumWeights](auto const& pair) -> std::pair<Id, double> {
                     return {pair.first, pair.second / sumWeights};
                   });
  }
  void FirstOrderDynamics::setDestinationNodes(
      std::unordered_map<Id, double> const& destinationNodes) {
    m_itineraries.clear();
    m_destinationNodes.clear();
    m_destinationNodes.reserve(destinationNodes.size());
    auto sumWeights{0.};
    std::for_each(destinationNodes.begin(),
                  destinationNodes.end(),
                  [this, &sumWeights](auto const& pair) -> void {
                    sumWeights += pair.second;
                    this->addItinerary(pair.first, pair.first);
                  });
    if (sumWeights <= 0.) {
      throw std::invalid_argument(
          std::format("The sum of the weights ({}) must be positive", sumWeights));
    }
    if (sumWeights == 1.) {
      std::copy(destinationNodes.begin(),
                destinationNodes.end(),
                std::back_inserter(m_destinationNodes));
      return;
    }
    std::transform(destinationNodes.begin(),
                   destinationNodes.end(),
                   std::back_inserter(m_destinationNodes),
                   [sumWeights](auto const& pair) -> std::pair<Id, double> {
                     return {pair.first, pair.second / sumWeights};
                   });
  }
  void FirstOrderDynamics::importODsFromCSV(std::string_view const fileName,
                                            char const separator) {
    csv::CSVFormat format;
    format.delimiter(separator);
    csv::CSVReader reader(fileName, format);

    auto const& colNames = reader.get_col_names();

    AgentInsertionMethod csvtype{AgentInsertionMethod::RANDOM};
    for (auto const& colName : colNames) {
      if (colName == "node_id") {
        csvtype = AgentInsertionMethod::RANDOM_ODS;
        break;
      } else if (colName == "origin_id") {
        csvtype = AgentInsertionMethod::ODS;
        break;
      }
    }

    switch (csvtype) {
      case AgentInsertionMethod::RANDOM_ODS: {
        spdlog::info("Importing ODs from CSV with RANDOM_ODS method.");
        std::unordered_map<Id, double> originNodes;
        std::unordered_map<Id, double> destinationNodes;
        for (auto const& row : reader) {
          auto const nodeId = row["node_id"].get<Id>();
          auto const type = row["type"].get<std::string>();
          auto const weight = row["weight"].get<double>();
          if (type == "O") {
            originNodes[nodeId] = weight;
          } else if (type == "D") {
            destinationNodes[nodeId] = weight;
          } else {
            spdlog::warn(
                "Unknown type '{}' for node {} in CSV. Skipping this row.", type, nodeId);
          }
        }
        this->setOriginNodes(std::move(originNodes));
        this->setDestinationNodes(std::move(destinationNodes));
        break;
      }
      case AgentInsertionMethod::ODS: {
        spdlog::info("Importing ODs from CSV with RANDOM_OD_PAIRS method.");
        std::vector<std::tuple<Id, Id, double>> ODs;
        for (auto const& row : reader) {
          auto const originId = row["origin_id"].get<Id>();
          auto const destinationId = row["destination_id"].get<Id>();
          auto const weight = row["weight"].get<double>();
          ODs.emplace_back(originId, destinationId, weight);
        }
        this->setODs(std::move(ODs));
        break;
      }
      default:
        throw std::runtime_error(
            "Could not determine the CSV type based on column names. Expected columns: "
            "'node_id' for RANDOM_ODS or 'origin_id' and 'destination_id' for "
            "RANDOM_OD_PAIRS.");
    }
  }
  void FirstOrderDynamics::initTurnCounts() {
    if (!m_turnCounts.empty()) {
      throw std::runtime_error("Turn counts have already been initialized.");
    }
    for (auto const& [edgeId, pEdge] : this->graph().edges()) {
      auto const* pTargetNode{&this->graph().node(pEdge->target())};
      for (auto const& nextEdgeId : pTargetNode->outgoingEdges()) {
        spdlog::debug("Initializing turn count for edge {} -> {}", edgeId, nextEdgeId);
        m_turnCounts[edgeId][nextEdgeId] = 0;
      }
    }
  }
  // You may wonder why not just use one function...
  // Never trust the user!
  // Jokes aside, the init is necessary because it allocates the memory for the first time and
  // turn counts are not incremented if the map is empty for performance reasons.
  void FirstOrderDynamics::resetTurnCounts() {
    if (m_turnCounts.empty()) {
      throw std::runtime_error("Turn counts have not been initialized.");
    }
    for (auto const& [edgeId, pEdge] : this->graph().edges()) {
      auto const* pTargetNode{&this->graph().node(pEdge->target())};
      for (auto const& nextEdgeId : pTargetNode->outgoingEdges()) {
        m_turnCounts[edgeId][nextEdgeId] = 0;
      }
    }
  }

  void FirstOrderDynamics::saveData(std::time_t const savingInterval,
                                    bool const saveAverageStats,
                                    bool const saveStreetData,
                                    bool const saveTravelData,
                                    bool const saveAgentData) {
    m_savingInterval = savingInterval;
    m_bSaveAverageStats = saveAverageStats;
    m_bSaveStreetData = saveStreetData;
    m_bSaveTravelData = saveTravelData;
    m_bSaveAgentData = saveAgentData;

    if (this->database() != nullptr) {
      // Initialize the required tables
      if (saveStreetData) {
        m_initStreetTable();
      }
      if (saveAverageStats) {
        m_initAvgStatsTable();
      }
      if (saveTravelData) {
        m_initTravelDataTable();
      }
      if (saveAgentData) {
        m_initAgentDataTable();
      }
      this->m_dumpSimInfo();
      this->m_dumpNetwork();
    }
    if (saveAgentData) {
      Street::acquireAgentData();
    }

    spdlog::info(
        "Data saving configured: interval={}s, avg_stats={}, street_data={}, "
        "travel_data={}, agent_data={}",
        savingInterval,
        saveAverageStats,
        saveStreetData,
        saveTravelData,
        saveAgentData);
  }

  void FirstOrderDynamics::setDestinationNodes(
      std::initializer_list<Id> destinationNodes) {
    m_itineraries.clear();
    auto const numNodes{destinationNodes.size()};
    m_destinationNodes.clear();
    m_destinationNodes.reserve(numNodes);
    std::for_each(destinationNodes.begin(),
                  destinationNodes.end(),
                  [this, &numNodes](auto const& nodeId) -> void {
                    this->m_destinationNodes.push_back({nodeId, 1. / numNodes});
                    this->addItinerary(nodeId, nodeId);
                  });
  }
  void FirstOrderDynamics::setODs(std::vector<std::tuple<Id, Id, double>> const& ODs) {
    m_ODs.clear();
    m_ODCumulativeWeights.clear();
    m_ODs.reserve(ODs.size());
    auto const sumWeights = std::accumulate(
        ODs.begin(), ODs.end(), 0., [this](double sum, auto const& tuple) {
          // Add itineraries while summing weights
          if (!this->itineraries().contains(std::get<1>(tuple))) {
            this->addItinerary(std::get<1>(tuple), std::get<1>(tuple));
          }
          return sum + std::get<2>(tuple);
        });
    if (sumWeights <= 0.) {
      throw std::invalid_argument(
          std::format("The sum of the weights ({}) must be positive", sumWeights));
    }
    if (sumWeights == 1.) {
      std::copy(ODs.begin(), ODs.end(), std::back_inserter(m_ODs));
    } else {
      // Copy but divide by weights sum
      std::transform(ODs.begin(),
                     ODs.end(),
                     std::back_inserter(m_ODs),
                     [sumWeights](auto const& tuple) {
                       return std::make_tuple(std::get<0>(tuple),
                                              std::get<1>(tuple),
                                              std::get<2>(tuple) / sumWeights);
                     });
    }
  }

  void FirstOrderDynamics::updatePaths(bool const throw_on_empty) {
    spdlog::debug("Init updating paths...");
    tbb::concurrent_vector<Id> emptyItineraries;
    tbb::parallel_for_each(
        this->itineraries().cbegin(),
        this->itineraries().cend(),
        [this, throw_on_empty, &emptyItineraries](auto const& pair) -> void {
          auto const& pItinerary{pair.second};
          this->m_updatePath(pItinerary);
          if (pItinerary->empty()) {
            if (!throw_on_empty) {
              spdlog::warn("No path found for itinerary {} with destination node {}",
                           pItinerary->id(),
                           pItinerary->destination());
              emptyItineraries.push_back(pItinerary->id());
              return;
            }
            throw std::runtime_error(
                std::format("No path found for itinerary {} with destination node {}",
                            pItinerary->id(),
                            pItinerary->destination()));
          }
        });
    if (!emptyItineraries.empty()) {
      spdlog::warn("Removing {} itineraries with no valid path from the dynamics.",
                   emptyItineraries.size());
      for (auto const& id : emptyItineraries) {
        auto const destination = m_itineraries.at(id)->destination();
        std::erase_if(m_ODs, [destination](auto const& tuple) {
          return std::get<1>(tuple) == destination;
        });
        std::erase_if(m_destinationNodes, [destination](auto const& tuple) {
          return std::get<0>(tuple) == destination;
        });
        std::erase_if(m_originNodes, [destination](auto const& tuple) {
          return std::get<0>(tuple) == destination;
        });
        m_itineraries.erase(id);
      }
    }
    spdlog::debug("End updating paths.");
  }

  void FirstOrderDynamics::addAgentsUniformly(std::size_t nAgents,
                                              std::optional<Id> optItineraryId) {
    m_nAddedAgents += nAgents;
    if (m_timeToleranceFactor.has_value() && !m_agents.empty()) {
      auto const nStagnantAgents{m_agents.size()};
      spdlog::debug(
          "Removing {} stagnant agents that were not inserted since the previous call to "
          "addAgentsUniformly().",
          nStagnantAgents);
      m_agents.clear();
      m_nAgents -= nStagnantAgents;
    }
    if (optItineraryId.has_value() && !this->itineraries().contains(*optItineraryId)) {
      throw std::invalid_argument(
          std::format("No itineraries available. Cannot add agents with itinerary id {}",
                      optItineraryId.value()));
    }
    bool const bRandomItinerary{!optItineraryId.has_value() &&
                                !this->itineraries().empty()};
    std::shared_ptr<Itinerary> pItinerary;
    std::uniform_int_distribution<std::size_t> itineraryDist{
        0, this->itineraries().size() - 1};
    std::uniform_int_distribution<std::size_t> streetDist{0, this->graph().nEdges() - 1};
    if (this->nAgents() + nAgents > this->graph().capacity()) {
      throw std::overflow_error(std::format(
          "Cannot add {} agents. The graph has currently {} with a maximum capacity of "
          "{}.",
          nAgents,
          this->nAgents(),
          this->graph().capacity()));
    }
    for (std::size_t i{0}; i < nAgents; ++i) {
      if (bRandomItinerary) {
        auto itineraryIt{this->itineraries().cbegin()};
        std::advance(itineraryIt, itineraryDist(this->m_generator));
        pItinerary = itineraryIt->second;
      }
      auto streetIt = this->graph().edges().begin();
      while (true) {
        auto step = streetDist(this->m_generator);
        std::advance(streetIt, step);
        if (!(streetIt->second->isFull())) {
          break;
        }
        streetIt = this->graph().edges().begin();
      }
      auto const& street{streetIt->second};
      this->addAgent(pItinerary, street->source());
      auto& pAgent{this->m_agents.back()};
      pAgent->setStreetId(street->id());
      pAgent->setSpeed(this->m_speedFunction(*streetIt->second));
      pAgent->setFreeTime(this->time_step() +
                          std::ceil(street->length() / pAgent->speed()));
      street->addAgent(std::move(pAgent), this->time_step());
      this->m_agents.pop_back();
    }
  }

  void FirstOrderDynamics::addAgent(std::unique_ptr<Agent> pAgent) {
    m_agents.push_back(std::move(pAgent));
    ++m_nAgents;
    ++m_nInsertedAgents;
    spdlog::trace("Added {}", *m_agents.back());
    auto const& optNodeId{m_agents.back()->srcNodeId()};
    if (optNodeId.has_value()) {
      auto [it, bInserted] = m_originCounts.insert({*optNodeId, 1});
      if (!bInserted) {
        ++it->second;
      }
    }
  }

  void FirstOrderDynamics::addAgents(std::size_t const nAgents,
                                     AgentInsertionMethod const mode) {
    switch (mode) {
      case AgentInsertionMethod::RANDOM:
        this->m_addAgentsRandom(nAgents);
        break;
      case AgentInsertionMethod::ODS:
        this->m_addAgentsODs(nAgents);
        break;
      case AgentInsertionMethod::RANDOM_ODS:
        this->m_addAgentsRandomODs(nAgents);
        break;
      default:
        throw std::runtime_error(
            "Cannot add agents without a valid insertion methods. Possible values are "
            "\"RANDOM\", \"ODS\" and \"RANDOM_ODS\"");
    }
  }

  void FirstOrderDynamics::addItinerary(std::shared_ptr<Itinerary> itinerary) {
    if (m_itineraries.contains(itinerary->id())) {
      throw std::invalid_argument(
          std::format("Itinerary with id {} already exists.", itinerary->id()));
    }
    m_itineraries.emplace(itinerary->id(), std::move(itinerary));
  }

  void FirstOrderDynamics::evolve(bool const reinsert_agents) {
    auto const n_threads{std::max<std::size_t>(1, this->concurrency())};
    std::atomic<double> mean_speed{0.}, mean_density{0.}, mean_traveltime{0.},
        mean_queue_length{0.};
    std::atomic<double> std_speed{0.}, std_density{0.};
    std::atomic<std::size_t> nValidEdges{0};
    bool const bComputeStats = m_savingInterval.has_value() &&
                               (m_savingInterval.value() == 0 ||
                                this->time_step() % m_savingInterval.value() == 0);
    tbb::concurrent_map<Id, StreetDataRecord> streetDataRecords;

    spdlog::debug("Init evolve at time {}", this->time_step());
    // move the first agent of each street queue, if possible, putting it in the next node
    bool const bUpdateData = m_dataUpdatePeriod.has_value() &&
                             this->time_step() % m_dataUpdatePeriod.value() == 0;
    auto const numNodes{this->graph().nNodes()};
    auto const numEdges{this->graph().nEdges()};

    const auto grainSize = std::max<std::size_t>(1, numNodes / (n_threads * 8));
    this->m_taskArena.execute([&] {
      tbb::parallel_for(
          tbb::blocked_range<std::size_t>(0, numNodes, grainSize),
          [&](const tbb::blocked_range<std::size_t>& range) {
            for (std::size_t i = range.begin(); i != range.end(); ++i) {
              auto* pNode = &this->graph().node(m_nodeIndices[i]);
              for (auto const& inEdgeId : pNode->ingoingEdges()) {
                auto* pStreet{&this->graph().edge(inEdgeId)};
                if (bUpdateData && pNode->isTrafficLight()) {
                  if (!m_queuesAtTrafficLights.contains(inEdgeId)) {
                    auto& tl = dynamic_cast<TrafficLight&>(*pNode);
                    assert(!tl.cycles().empty());
                    for (auto const& [id, pair] : tl.cycles()) {
                      for (auto const& [direction, cycle] : pair) {
                        m_queuesAtTrafficLights[id].emplace(direction, 0.);
                      }
                    }
                  }
                  for (auto& [direction, value] : m_queuesAtTrafficLights.at(inEdgeId)) {
                    value += pStreet->nExitingAgents(direction, true);
                  }
                }
                m_evolveStreet(pStreet, reinsert_agents);
                if (bComputeStats) {
                  auto const& density{pStreet->density() * 1e3};
                  auto const& queueLength{pStreet->nExitingAgents()};

                  auto const speedMeasure = pStreet->meanSpeed(true);
                  if (speedMeasure.is_valid) {
                    auto const speed = speedMeasure.mean * 3.6;  // to kph
                    auto const speed_std = speedMeasure.std * 3.6;
                    if (m_bSaveAverageStats) {
                      mean_speed.fetch_add(speed, std::memory_order_relaxed);
                      std_speed.fetch_add(speed * speed + speed_std * speed_std,
                                          std::memory_order_relaxed);
                      mean_traveltime.fetch_add(pStreet->length() / speedMeasure.mean,
                                                std::memory_order_relaxed);
                      ++nValidEdges;
                    }
                  }
                  if (m_bSaveAverageStats) {
                    mean_density.fetch_add(density, std::memory_order_relaxed);
                    std_density.fetch_add(density * density, std::memory_order_relaxed);
                    mean_queue_length.fetch_add(queueLength, std::memory_order_relaxed);
                  }

                  if (m_bSaveStreetData) {
                    // Collect data for batch insert after parallel section
                    StreetDataRecord record;
                    record.density = density;
                    if (pStreet->hasCoil()) {
                      record.coilName = pStreet->counterName();
                      record.counts = pStreet->counts();
                      pStreet->resetCounter();
                    }
                    if (speedMeasure.is_valid) {
                      record.avgSpeed = speedMeasure.mean * 3.6;  // to kph
                      record.stdSpeed = speedMeasure.std * 3.6;
                      record.nObservations = speedMeasure.n;
                    }
                    record.queueLength = queueLength;
                    streetDataRecords[pStreet->id()] = std::move(record);
                  }
                }
              }
            }
          },
          tbb::auto_partitioner{});
    });
    spdlog::debug("Pre-nodes");
    // Move transport capacity agents from each node
    this->m_taskArena.execute([&] {
      tbb::parallel_for(
          tbb::blocked_range<size_t>(0, numNodes, grainSize),
          [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
              auto* pNode = &this->graph().node(m_nodeIndices[i]);
              m_evolveNode(pNode);
              if (pNode->isTrafficLight()) {
                auto& tl = dynamic_cast<TrafficLight&>(*pNode);
                ++tl;
              }
            }
          },
          tbb::auto_partitioner{});
    });
    this->m_evolveAgents();

    if (bComputeStats) {
      auto const datetime = this->strDateTime();
      auto const step = static_cast<std::int64_t>(this->time_step());
      auto const simulationId = static_cast<std::int64_t>(this->id());

      bool const hasWritePayload = (m_bSaveStreetData && !streetDataRecords.empty()) ||
                                   (m_bSaveTravelData && !m_travelDTs.empty()) ||
                                   m_bSaveAverageStats;
      if (this->database() != nullptr) {
        spdlog::debug("Saving data for time step {} to the database...",
                      this->time_step());
        std::optional<SQLite::Transaction> transaction;
        if (hasWritePayload) {
          transaction.emplace(*this->database());
        }
        // Batch insert street data collected during parallel section
        if (m_bSaveStreetData) {
          this->m_saveStreetDataSQL(
              datetime, step, simulationId, std::move(streetDataRecords));
        }
        if (m_bSaveTravelData && !m_travelDTs.empty()) {
          this->m_saveTravelDataSQL(datetime, step, simulationId, std::move(m_travelDTs));
          m_travelDTs.clear();
        }
        if (m_bSaveAgentData) {
          this->m_saveAgentDataSQL(step, simulationId, Street::agentData());
        }

        if (m_bSaveAverageStats) {  // Average Stats Table
          double meanSpeed{0.}, stdSpeed{0.}, meanDensity{0.}, stdDensity{0.},
              meanTravelTime{0.}, meanQueueLength{0.};
          auto const validEdges = nValidEdges.load();
          auto const edgeCount = static_cast<double>(numEdges);
          if (validEdges > 0) {
            meanSpeed = mean_speed.load() / validEdges;
            stdSpeed = std::sqrt(
                std::max(0.0, std_speed.load() / validEdges - meanSpeed * meanSpeed));
            meanDensity = mean_density.load() / edgeCount;
            stdDensity = std::sqrt(std::max(
                0.0, std_density.load() / edgeCount - meanDensity * meanDensity));
            meanTravelTime = mean_traveltime.load() / validEdges;
            meanQueueLength = mean_queue_length.load() / edgeCount;
          }

          this->m_saveAvgStatsSQL(datetime,
                                  step,
                                  simulationId,
                                  validEdges,
                                  meanSpeed,
                                  stdSpeed,
                                  meanDensity,
                                  stdDensity,
                                  meanTravelTime,
                                  meanQueueLength);
        }

        if (transaction.has_value()) {
          transaction->commit();
        }
      } else {
        spdlog::debug("No database connected. Saving csv files for time step {}...",
                      this->time_step());
        if (m_bSaveStreetData) {
          this->m_saveStreetDataCSV(
              datetime, step, simulationId, std::move(streetDataRecords));
        }
        if (m_bSaveTravelData && !m_travelDTs.empty()) {
          this->m_saveTravelDataCSV(datetime, step, simulationId, std::move(m_travelDTs));
          m_travelDTs.clear();
        }
        if (m_bSaveAgentData) {
          this->m_saveAgentDataCSV(step, simulationId, Street::agentData());
        }
        if (m_bSaveAverageStats) {  // Average Stats Table
          double meanSpeed{0.}, stdSpeed{0.}, meanDensity{0.}, stdDensity{0.},
              meanTravelTime{0.}, meanQueueLength{0.};
          auto const validEdges = nValidEdges.load();
          auto const edgeCount = static_cast<double>(numEdges);
          if (validEdges > 0) {
            meanSpeed = mean_speed.load() / validEdges;
            stdSpeed = std::sqrt(
                std::max(0.0, std_speed.load() / validEdges - meanSpeed * meanSpeed));
            meanDensity = mean_density.load() / edgeCount;
            stdDensity = std::sqrt(std::max(
                0.0, std_density.load() / edgeCount - meanDensity * meanDensity));
            meanTravelTime = mean_traveltime.load() / validEdges;
            meanQueueLength = mean_queue_length.load() / edgeCount;
          }
          this->m_saveAvgStatsCSV(datetime,
                                  step,
                                  simulationId,
                                  validEdges,
                                  meanSpeed,
                                  stdSpeed,
                                  meanDensity,
                                  stdDensity,
                                  meanTravelTime,
                                  meanQueueLength);
        }
      }

      // Special case: if m_savingInterval == 0, it was a triggered saveData() call, so we need to reset all flags
      if (m_savingInterval.value() == 0) {
        m_savingInterval.reset();
        m_bSaveStreetData = false;
        m_bSaveTravelData = false;
        m_bSaveAverageStats = false;
        m_bSaveAgentData = false;
      }
    }

    Dynamics<RoadNetwork>::m_evolve();
  }

  void FirstOrderDynamics::m_trafficlightSingleTailOptimizer(
      double const& beta, std::optional<std::ofstream>& logStream) {
    assert(beta >= 0. && beta <= 1.);
    if (logStream.has_value()) {
      *logStream << std::format(
          "Init Traffic Lights optimization (SINGLE TAIL) - Time {} - Alpha {}\n",
          this->time_step(),
          beta);
    }
    for (auto const& [nodeId, pNode] : this->graph().nodes()) {
      if (!pNode->isTrafficLight()) {
        continue;
      }
      auto& tl = dynamic_cast<TrafficLight&>(*pNode);

      auto const& inNeighbours{pNode->ingoingEdges()};

      // Default is RIGHTANDSTRAIGHT - LEFT phases for both priority and non-priority
      std::array<double, 2> inputPrioritySum{0., 0.}, inputNonPrioritySum{0., 0.};
      bool isPrioritySinglePhase{false}, isNonPrioritySinglePhase{false};

      for (const auto& streetId : inNeighbours) {
        if (tl.cycles().at(streetId).contains(Direction::ANY)) {
          tl.streetPriorities().contains(streetId) ? isPrioritySinglePhase = true
                                                   : isNonPrioritySinglePhase = true;
        }
      }
      if (isPrioritySinglePhase && logStream.has_value()) {
        *logStream << "\tFound a single phase for priority streets.\n";
      }
      if (isNonPrioritySinglePhase && logStream.has_value()) {
        *logStream << "\tFound a single phase for non-priority streets.\n";
      }

      for (const auto& streetId : inNeighbours) {
        for (auto const& [direction, tail] : m_queuesAtTrafficLights.at(streetId)) {
          if (tl.streetPriorities().contains(streetId)) {
            if (isPrioritySinglePhase) {
              inputPrioritySum[0] += tail;
            } else {
              if (direction == Direction::LEFT ||
                  direction == Direction::LEFTANDSTRAIGHT) {
                inputPrioritySum[1] += tail;
              } else {
                inputPrioritySum[0] += tail;
              }
            }
          } else {
            if (isNonPrioritySinglePhase) {
              inputNonPrioritySum[0] += tail;
            } else {
              if (direction == Direction::LEFT ||
                  direction == Direction::LEFTANDSTRAIGHT) {
                inputNonPrioritySum[1] += tail;
              } else {
                inputNonPrioritySum[0] += tail;
              }
            }
          }
        }
      }
      {
        // Sum normalization
        auto const sum{inputPrioritySum[0] + inputPrioritySum[1] +
                       inputNonPrioritySum[0] + inputNonPrioritySum[1]};
        if (sum == 0.) {
          continue;
        }
        inputPrioritySum[0] /= sum;
        inputPrioritySum[1] /= sum;
        inputNonPrioritySum[0] /= sum;
        inputNonPrioritySum[1] /= sum;

        // int const cycleTime{(1. - alpha) * tl.cycleTime()};

        inputPrioritySum[0] *= beta;
        inputPrioritySum[1] *= beta;
        inputNonPrioritySum[0] *= beta;
        inputNonPrioritySum[1] *= beta;
      }

      if (logStream.has_value()) {
        *logStream << std::format(
            "\tInput cycle queue ratios are {:.2f} {:.2f} - {:.2f} {:.2f}\n",
            inputPrioritySum[0],
            inputPrioritySum[1],
            inputNonPrioritySum[0],
            inputNonPrioritySum[1]);
      }

      tl.resetCycles();
      auto cycles{tl.cycles()};
      std::array<int, 4> n{0, 0, 0, 0};
      std::array<double, 4> greenTimes{0., 0., 0., 0.};

      for (auto const& [streetId, pair] : cycles) {
        for (auto const& [direction, cycle] : pair) {
          if (tl.streetPriorities().contains(streetId)) {
            if (isPrioritySinglePhase) {
              greenTimes[0] += cycle.greenTime();
              ++n[0];
            } else {
              if (direction == Direction::LEFT ||
                  direction == Direction::LEFTANDSTRAIGHT) {
                greenTimes[1] += cycle.greenTime();
                ++n[1];
              } else {
                greenTimes[0] += cycle.greenTime();
                ++n[0];
              }
            }
          } else {
            if (isNonPrioritySinglePhase) {
              greenTimes[2] += cycle.greenTime();
              ++n[2];
            } else {
              if (direction == Direction::LEFT ||
                  direction == Direction::LEFTANDSTRAIGHT) {
                greenTimes[3] += cycle.greenTime();
                ++n[3];
              } else {
                greenTimes[2] += cycle.greenTime();
                ++n[2];
              }
            }
          }
        }
      }

      if (logStream.has_value()) {
        *logStream << std::format("\tGreen times are {} {} - {} {}\n",
                                  greenTimes[0],
                                  greenTimes[1],
                                  greenTimes[2],
                                  greenTimes[3]);
      }

      for (auto i{0}; i < 4; ++i) {
        if (n[i] > 1) {
          greenTimes[i] /= n[i];
        }
      }

      {
        auto sum{0.};
        for (auto const& greenTime : greenTimes) {
          sum += greenTime;
        }
        if (sum == 0.) {
          continue;
        }
        for (auto& greenTime : greenTimes) {
          greenTime /= sum;
        }
      }
      for (auto& el : greenTimes) {
        el *= (1. - beta);
      }

      int inputPriorityR{static_cast<int>(
          std::floor((inputPrioritySum[0] + greenTimes[0]) * tl.cycleTime()))};
      int inputPriorityS{inputPriorityR};
      int inputPriorityL{static_cast<int>(
          std::floor((inputPrioritySum[1] + greenTimes[1]) * tl.cycleTime()))};

      int inputNonPriorityR{static_cast<int>(
          std::floor((inputNonPrioritySum[0] + greenTimes[2]) * tl.cycleTime()))};
      int inputNonPriorityS{inputNonPriorityR};
      int inputNonPriorityL{static_cast<int>(
          std::floor((inputNonPrioritySum[1] + greenTimes[3]) * tl.cycleTime()))};

      {
        // Adjust phases to have the sum equal to the cycle time
        // To do this, first add seconds to the priority streets, then to the
        // non-priority streets
        auto total{static_cast<Delay>(inputPriorityR + inputPriorityL +
                                      inputNonPriorityR + inputNonPriorityL)};
        size_t idx{0};
        while (total < tl.cycleTime()) {
          switch (idx % 4) {
            case 0:
              ++inputPriorityR;
              ++inputPriorityS;
              break;
            case 1:
              ++inputPriorityL;
              break;
            case 2:
              ++inputNonPriorityR;
              ++inputNonPriorityS;
              break;
            case 3:
              ++inputNonPriorityL;
              break;
          }
          ++idx;
          ++total;
        }
      }

      if (isPrioritySinglePhase) {
        inputPriorityR = 0;
        inputPriorityL = 0;
      }
      if (isNonPrioritySinglePhase) {
        inputNonPriorityR = 0;
        inputNonPriorityL = 0;
      }

      // Logger::info(std::format(
      //     "Cycle time: {} - Current sum: {}",
      //     tl.cycleTime(),
      //     inputPriorityRS + inputPriorityL + inputNonPriorityRS + inputNonPriorityL));
      assert(inputPriorityS + inputPriorityL + inputNonPriorityS + inputNonPriorityL ==
             tl.cycleTime());

      std::unordered_map<Direction, TrafficLightCycle> priorityCycles;
      priorityCycles.emplace(Direction::RIGHT,
                             TrafficLightCycle{static_cast<Delay>(inputPriorityR), 0});
      priorityCycles.emplace(Direction::STRAIGHT,
                             TrafficLightCycle{static_cast<Delay>(inputPriorityS), 0});
      priorityCycles.emplace(Direction::RIGHTANDSTRAIGHT,
                             TrafficLightCycle{static_cast<Delay>(inputPriorityS), 0});
      priorityCycles.emplace(
          Direction::ANY,
          TrafficLightCycle{static_cast<Delay>(inputPriorityS + inputPriorityL), 0});
      priorityCycles.emplace(Direction::LEFT,
                             TrafficLightCycle{static_cast<Delay>(inputPriorityL),
                                               static_cast<Delay>(inputPriorityS)});

      std::unordered_map<Direction, TrafficLightCycle> nonPriorityCycles;
      nonPriorityCycles.emplace(
          Direction::RIGHT,
          TrafficLightCycle{static_cast<Delay>(inputNonPriorityR),
                            static_cast<Delay>(inputPriorityS + inputPriorityL)});
      nonPriorityCycles.emplace(
          Direction::STRAIGHT,
          TrafficLightCycle{static_cast<Delay>(inputNonPriorityS),
                            static_cast<Delay>(inputPriorityS + inputPriorityL)});
      nonPriorityCycles.emplace(
          Direction::RIGHTANDSTRAIGHT,
          TrafficLightCycle{static_cast<Delay>(inputNonPriorityS),
                            static_cast<Delay>(inputPriorityS + inputPriorityL)});
      nonPriorityCycles.emplace(
          Direction::ANY,
          TrafficLightCycle{static_cast<Delay>(inputNonPriorityS + inputNonPriorityL),
                            static_cast<Delay>(inputPriorityS + inputPriorityL)});
      nonPriorityCycles.emplace(
          Direction::LEFT,
          TrafficLightCycle{
              static_cast<Delay>(inputNonPriorityL),
              static_cast<Delay>(inputPriorityS + inputPriorityL + inputNonPriorityS)});
      nonPriorityCycles.emplace(
          Direction::LEFTANDSTRAIGHT,
          TrafficLightCycle{
              static_cast<Delay>(inputNonPriorityL + inputNonPriorityS),
              static_cast<Delay>(inputPriorityS + inputPriorityL + inputNonPriorityR)});

      std::vector<Id> streetIds;
      std::set<Id> forbiddenLeft;

      for (auto const& pair : cycles) {
        streetIds.push_back(pair.first);
      }
      for (auto const streetId : streetIds) {
        auto* pStreet{&this->graph().edge(streetId)};
        if (tl.streetPriorities().contains(streetId)) {
          for (auto& [dir, cycle] : cycles.at(streetId)) {
            if (isPrioritySinglePhase) {
              cycle = priorityCycles.at(Direction::STRAIGHT);
            } else {
              cycle = priorityCycles.at(dir);
            }
          }
          if (cycles.at(streetId).contains(Direction::RIGHT) &&
              cycles.at(streetId).contains(Direction::STRAIGHT)) {
            TrafficLightCycle freecycle{
                static_cast<Delay>(inputPriorityS + inputPriorityL), 0};
            // Logger::info(std::format("Free cycle (RIGHT) for {} -> {}: {} {}",
            //                          pStreet->source(),
            //                          pStreet->target(),
            //                          freecycle.greenTime(),
            //                          freecycle.phase()));
            cycles.at(streetId).at(Direction::RIGHT) = freecycle;
          }
        } else {
          for (auto& [dir, cycle] : cycles.at(streetId)) {
            if (isNonPrioritySinglePhase) {
              cycle = nonPriorityCycles.at(Direction::STRAIGHT);
            } else {
              cycle = nonPriorityCycles.at(dir);
            }
          }
          if (cycles.at(streetId).contains(Direction::RIGHT) &&
              cycles.at(streetId).contains(Direction::STRAIGHT)) {
            TrafficLightCycle freecycle{
                static_cast<Delay>(inputNonPriorityS + inputNonPriorityL),
                static_cast<Delay>(inputPriorityS + inputPriorityL)};
            // Logger::info(std::format("Free cycle (RIGHT) for {} -> {}: {} {}",
            //                          pStreet->source(),
            //                          pStreet->target(),
            //                          freecycle.greenTime(),
            //                          freecycle.phase()));
            cycles.at(streetId).at(Direction::RIGHT) = freecycle;
          }
        }
        bool found{false};
        for (auto const dir : pStreet->laneMapping()) {
          if (dir == Direction::LEFT || dir == Direction::LEFTANDSTRAIGHT ||
              dir == Direction::ANY) {
            found = true;
            break;
          }
        }
        if (!found) {
          forbiddenLeft.insert(streetId);
          // Logger::info(std::format("Street {} -> {} has forbidden left turn.",
          //                          pStreet->source(),
          //                          pStreet->target()));
        }
      }
      for (auto const forbiddenLeftStreetId : forbiddenLeft) {
        for (auto const streetId : streetIds) {
          if (streetId == forbiddenLeftStreetId) {
            continue;
          }
          if (tl.streetPriorities().contains(streetId) &&
              tl.streetPriorities().contains(forbiddenLeftStreetId)) {
            TrafficLightCycle freecycle{
                static_cast<Delay>(inputPriorityS + inputPriorityL), 0};
            for (auto& [direction, cycle] : cycles.at(streetId)) {
              if (direction == Direction::RIGHT || direction == Direction::STRAIGHT ||
                  direction == Direction::RIGHTANDSTRAIGHT) {
                auto* pStreet{&this->graph().edge(streetId)};
                if (logStream.has_value()) {
                  *logStream << std::format("\tFree cycle for {} -> {}: dir {} - {}\n",
                                            pStreet->source(),
                                            pStreet->target(),
                                            directionToString[direction],
                                            freecycle);
                }
                cycle = freecycle;
              }
            }
          } else if (!tl.streetPriorities().contains(streetId) &&
                     !tl.streetPriorities().contains(forbiddenLeftStreetId)) {
            TrafficLightCycle freecycle{
                static_cast<Delay>(inputNonPriorityS + inputNonPriorityL),
                static_cast<Delay>(inputPriorityS + inputPriorityL)};
            for (auto& [direction, cycle] : cycles.at(streetId)) {
              if (direction == Direction::RIGHT || direction == Direction::STRAIGHT ||
                  direction == Direction::RIGHTANDSTRAIGHT) {
                auto* pStreet{&this->graph().edge(streetId)};
                if (logStream.has_value()) {
                  *logStream << std::format("Free cycle ({}) for {} -> {}: {} {}\n",
                                            directionToString[direction],
                                            pStreet->source(),
                                            pStreet->target(),
                                            freecycle.greenTime(),
                                            freecycle.phase());
                }
                cycle = freecycle;
              }
            }
          }
        }
      }

      tl.setCycles(cycles);
      if (logStream.has_value()) {
        *logStream << std::format("\nNew cycles for {}", tl);
      }
    }
    if (logStream.has_value()) {
      *logStream << std::format("End Traffic Lights optimization - Time {}\n",
                                this->time_step());
    }
  }

  void FirstOrderDynamics::optimizeTrafficLights(
      TrafficLightOptimization const optimizationType,
      const std::string& logFile,
      double const percentage,
      double const threshold) {
    std::optional<std::ofstream> logStream;
    if (!logFile.empty()) {
      logStream.emplace(logFile, std::ios::app);
      if (!logStream->is_open()) {
        spdlog::error("Could not open log file: {}", logFile);
      }
    }
    this->m_trafficlightSingleTailOptimizer(percentage, logStream);
    if (optimizationType == TrafficLightOptimization::DOUBLE_TAIL) {
      // Try to synchronize congested traffic lights
      std::unordered_map<Id, double> densities;
      for (auto const& [nodeId, pNode] : this->graph().nodes()) {
        if (!pNode->isTrafficLight()) {
          continue;
        }
        double density{0.}, n{0.};
        auto const& inNeighbours{pNode->ingoingEdges()};
        for (auto const& inEdgeId : inNeighbours) {
          auto* pStreet{&this->graph().edge(inEdgeId)};
          auto* pSourceNode{&this->graph().node(pStreet->source())};
          if (!pSourceNode->isTrafficLight()) {
            continue;
          }
          density += pStreet->density(true);
          ++n;
        }
        density /= n;
        densities[nodeId] = density;
      }
      // Sort densities map from big to small values
      std::vector<std::pair<Id, double>> sortedDensities(densities.begin(),
                                                         densities.end());

      // Sort by density descending
      std::sort(sortedDensities.begin(),
                sortedDensities.end(),
                [](auto const& a, auto const& b) { return a.second > b.second; });
      std::unordered_set<Id> optimizedNodes;

      for (auto const& [nodeId, density] : sortedDensities) {
        auto const& inNeighbours{this->graph().node(nodeId).ingoingEdges()};
        for (auto const& inEdgeId : inNeighbours) {
          auto* pStreet{&this->graph().edge(inEdgeId)};
          auto const& sourceId{pStreet->source()};
          if (!densities.contains(sourceId) || optimizedNodes.contains(sourceId)) {
            continue;
          }
          auto const& neighbourDensity{densities.at(sourceId)};
          if (neighbourDensity < threshold * density) {
            continue;
          }
          // Try to green-wave the situation
          auto& tl{dynamic_cast<TrafficLight&>(this->graph().node(sourceId))};
          tl.increasePhases(pStreet->length() /
                            (pStreet->maxSpeed() * (1. - 0.6 * pStreet->density(true))));
          optimizedNodes.insert(sourceId);
          if (logStream.has_value()) {
            *logStream << std::format("\nNew cycles for {}", tl);
          }
        }
      }
    }
    // Cleaning variables
    for (auto& [streetId, pair] : m_queuesAtTrafficLights) {
      for (auto& [direction, value] : pair) {
        value = 0.;
      }
    }
    m_previousOptimizationTime = this->time_step();
    if (logStream.has_value()) {
      logStream->close();
    }
  }

  Measurement<double> FirstOrderDynamics::meanTravelTime(bool clearData) {
    std::vector<double> travelTimes;
    if (!m_travelDTs.empty()) {
      travelTimes.reserve(m_travelDTs.size());
      for (auto const& [distance, time] : m_travelDTs) {
        travelTimes.push_back(time);
      }
      if (clearData) {
        m_travelDTs.clear();
      }
    }
    return Measurement<double>(travelTimes);
  }
  Measurement<double> FirstOrderDynamics::meanTravelDistance(bool clearData) {
    if (m_travelDTs.empty()) {
      return Measurement<double>();
    }
    std::vector<double> travelDistances;
    travelDistances.reserve(m_travelDTs.size());
    for (auto const& [distance, time] : m_travelDTs) {
      travelDistances.push_back(distance);
    }
    if (clearData) {
      m_travelDTs.clear();
    }
    return Measurement<double>(travelDistances);
  }
  Measurement<double> FirstOrderDynamics::meanTravelSpeed(bool clearData) {
    std::vector<double> travelSpeeds;
    travelSpeeds.reserve(m_travelDTs.size());
    for (auto const& [distance, time] : m_travelDTs) {
      travelSpeeds.push_back(distance / time);
    }
    if (clearData) {
      m_travelDTs.clear();
    }
    return Measurement<double>(travelSpeeds);
  }
  std::unordered_map<Id, std::unordered_map<Id, double>> const
  FirstOrderDynamics::normalizedTurnCounts() const noexcept {
    std::unordered_map<Id, std::unordered_map<Id, double>> normalizedTurnCounts;
    for (auto const& [fromId, map] : m_turnCounts) {
      auto const sum{
          std::accumulate(map.begin(), map.end(), 0., [](auto const sum, auto const& p) {
            return sum + static_cast<double>(p.second);
          })};
      for (auto const& [toId, count] : map) {
        normalizedTurnCounts[fromId][toId] =
            sum == 0. ? 0. : static_cast<double>(count) / sum;
      }
    }
    return normalizedTurnCounts;
  }

  tbb::concurrent_unordered_map<Id, std::size_t> FirstOrderDynamics::originCounts(
      bool const bReset) noexcept {
    if (!bReset) {
      return m_originCounts;
    }
    auto const tempCounts{std::move(m_originCounts)};
    m_originCounts.clear();
    return tempCounts;
  }
  tbb::concurrent_unordered_map<Id, std::size_t> FirstOrderDynamics::destinationCounts(
      bool const bReset) noexcept {
    if (!bReset) {
      return m_destinationCounts;
    }
    auto const tempCounts{std::move(m_destinationCounts)};
    m_destinationCounts.clear();
    return tempCounts;
  }

  Measurement<double> FirstOrderDynamics::streetMeanDensity(bool normalized) const {
    if (this->graph().edges().empty()) {
      return Measurement<double>();
    }
    std::vector<double> densities;
    densities.reserve(this->graph().nEdges());
    if (normalized) {
      for (const auto& [streetId, pStreet] : this->graph().edges()) {
        densities.push_back(pStreet->density(true));
      }
    } else {
      double sum{0.};
      for (const auto& [streetId, pStreet] : this->graph().edges()) {
        densities.push_back(pStreet->density(false) * pStreet->length());
        sum += pStreet->length();
      }
      if (sum == 0) {
        return Measurement<double>();
      }
      auto meanDensity{std::accumulate(densities.begin(), densities.end(), 0.) / sum};
      return Measurement<double>(meanDensity, 0., densities.size());
    }
    return Measurement<double>(densities);
  }

  Measurement<double> FirstOrderDynamics::streetMeanFlow() const {
    std::vector<double> flows;
    flows.reserve(this->graph().nEdges());
    for (const auto& [streetId, pStreet] : this->graph().edges()) {
      auto const speedMeasure = pStreet->meanSpeed();
      if (speedMeasure.is_valid) {
        flows.push_back(pStreet->density() * speedMeasure.mean);
      }
    }
    return Measurement<double>(flows);
  }

  Measurement<double> FirstOrderDynamics::streetMeanFlow(double threshold,
                                                         bool above) const {
    std::vector<double> flows;
    flows.reserve(this->graph().nEdges());
    for (const auto& [streetId, pStreet] : this->graph().edges()) {
      auto const speedMeasure = pStreet->meanSpeed();
      if (!speedMeasure.is_valid) {
        continue;
      }
      if (above && (pStreet->density(true) > threshold)) {
        flows.push_back(pStreet->density() * speedMeasure.mean);
      } else if (!above && (pStreet->density(true) < threshold)) {
        flows.push_back(pStreet->density() * speedMeasure.mean);
      }
    }
    return Measurement<double>(flows);
  }

  void FirstOrderDynamics::summary(std::ostream& os) const {
    os << "FirstOrderDynamics Summary:\n";
    this->graph().describe(os);
    os << "\nNumber of added agents: " << m_nAddedAgents << '\n'
       << "Number of inserted agents: " << m_nInsertedAgents << '\n'
       << "Number of arrived agents: " << m_nArrivedAgents << '\n'
       << "Number of killed agents: " << m_nKilledAgents << '\n'
       << "Current number of agents: " << this->nAgents() << '\n';
  }

}  // namespace dsf::mobility

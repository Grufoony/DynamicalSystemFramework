#include "FirstOrderDynamics.hpp"

#include <csv.hpp>
#include <SQLiteCpp/SQLiteCpp.h>
#include <spdlog/spdlog.h>

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
  void FirstOrderDynamics::m_addAgentsConditionalRandomODs(std::size_t nAgents) {
    m_nAddedAgents += nAgents;
    if (m_timeToleranceFactor.has_value() && !m_agents.empty()) {
      auto const nStagnantAgents{m_agents.size()};
      spdlog::debug(
          "Removing {} stagnant agents that were not inserted since the previous call to "
          "addAgentsConditionalRandomODs().",
          nStagnantAgents);
      m_agents.clear();
      m_nAgents -= nStagnantAgents;
    }
    if (m_originNodes.empty()) {
      throw std::runtime_error(
          "FirstOrderDynamics::m_addAgentsConditionalRandomODs: Origin nodes must be "
          "set");
    }
    std::uniform_real_distribution<double> uniformDist{0., 1.};
    spdlog::debug("Adding {} agents at time {}.", nAgents, this->time_step());
    while (nAgents--) {
      // Select origin using weighted random selection
      auto randValue = uniformDist(this->m_generator);
      Id originId{
          std::get<0>(m_originNodes.back())};  // fallback to last for numerical stability
      for (const auto& [id, weight] : m_originNodes) {
        if (randValue < weight) {
          originId = id;
          break;
        }
        randValue -= weight;
      }
      // Select destination conditionally based on the selected origin
      auto originToDestIt = m_originToDestinations.find(originId);
      if (originToDestIt == m_originToDestinations.end() ||
          originToDestIt->second.empty()) {
        throw std::runtime_error(std::format(
            "No destinations found for origin {} in conditional random OD insertion.",
            originId));
      }
      const auto& destinations = originToDestIt->second;
      randValue = uniformDist(this->m_generator);
      Id destinationId{
          std::get<0>(destinations.back())};  // fallback to last for numerical stability
      for (const auto& [id, weight] : destinations) {
        if (randValue < weight) {
          destinationId = id;
          break;
        }
        randValue -= weight;
      }
      // Find the itinerary with the given destination
      auto itineraryIt{std::find_if(this->itineraries().cbegin(),
                                    this->itineraries().cend(),
                                    [destinationId](const auto& itinerary) {
                                      return itinerary.second->destination() ==
                                             destinationId;
                                    })};
      if (itineraryIt == this->itineraries().cend()) {
        spdlog::error("Itinerary with destination {} not found. Skipping agent.",
                      destinationId);
        continue;
      }
      // Check if destination is reachable from origin
      auto const& itinerary = itineraryIt->second;
      if (!itinerary->path().contains(originId)) {
        spdlog::debug("Destination {} not reachable from origin {}. Skipping agent.",
                      destinationId,
                      originId);
        continue;
      }
      this->addAgent(itineraryIt->second, originId);
    }
  }

  std::optional<Id> FirstOrderDynamics::m_extractStreet(
      std::unordered_map<Id, double> const& transitionProbabilities,
      double const cumulativeProbability) {
    // Select street based on weighted probabilities
    if (transitionProbabilities.empty()) {
      return std::nullopt;
    }
    if (transitionProbabilities.size() == 1) {
      auto const& onlyStreetId = transitionProbabilities.cbegin()->first;
      spdlog::trace("This transition is to {}", this->graph().edge(onlyStreetId));
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
    return fallbackStreetId;
  }

  std::optional<Id> FirstOrderDynamics::m_nextRandomStreetId(
      const std::unique_ptr<Agent>& pAgent, RoadJunction const* pNode) {
    spdlog::trace("Computing m_nextRandomStreetId for {}", *pAgent);
    auto const& outgoingEdges = pNode->outgoingEdges();

    // Calculate transition probabilities for all valid outgoing edges
    std::unordered_map<Id, double> transitionProbabilities;
    double cumulativeProbability = 0.0;

    std::set<Id> forbiddenTurns;
    std::optional<Id> previousNodeId{std::nullopt};
    if (pAgent->streetId().has_value()) {
      auto const& streetCurrent{this->graph().edge(pAgent->streetId().value())};
      forbiddenTurns = streetCurrent.forbiddenTurns();
      previousNodeId = streetCurrent.source();
    }

    for (const auto outEdgeId : outgoingEdges) {
      if (forbiddenTurns.contains(outEdgeId)) {
        spdlog::trace("Forbidden turn from street {} to street {}. Skipping.",
                      pAgent->streetId().value_or(0),
                      outEdgeId);
        continue;
      }
      auto const& streetOut{this->graph().edge(outEdgeId)};

      double probability = 1.0;
      //std::exp(-0.5 * pStreetOut->maxSpeed() / (this->m_speedFunction(*pStreetOut)));

      // Handle U-turns
      if (previousNodeId.has_value() && streetOut.target() == previousNodeId.value()) {
        continue;
      }

      transitionProbabilities.emplace(streetOut.id(), probability);
      cumulativeProbability += probability;
    }
    spdlog::debug("Found {} valid transitions for {} at {}",
                  transitionProbabilities.size(),
                  *pAgent,
                  *pNode);
    return m_extractStreet(transitionProbabilities, cumulativeProbability);
  }

  std::optional<Id> FirstOrderDynamics::m_nextStreetId(
      const std::unique_ptr<Agent>& pAgent, RoadJunction const* pNode) {
    if (pAgent->isRandom()) {
      return m_nextRandomStreetId(pAgent, pNode);
    }
    spdlog::trace("Computing m_nextStreetId for {}", *pAgent);
    auto const& outgoingEdges = pNode->outgoingEdges();

    // Get current street information
    std::optional<Id> previousNodeId = std::nullopt;
    std::set<Id> forbiddenTurns;
    if (pAgent->streetId().has_value()) {
      auto const* pStreetCurrent{&this->graph().edge(pAgent->streetId().value())};
      previousNodeId = pStreetCurrent->source();
      forbiddenTurns = pStreetCurrent->forbiddenTurns();
    }

    // Get path targets for non-random agents
    std::vector<Id> pathTargets;
    auto const& path = pAgent->itinerary()->path();
    auto const pathIt = path.find(pNode->id());
    if (pathIt == path.cend()) {
      spdlog::debug("No itinerary path entry for {} at node {}. Returning no transition.",
                    *pAgent,
                    pNode->id());
      return std::nullopt;
    }
    pathTargets = pathIt->second;

    // Calculate transition probabilities for all valid outgoing edges
    std::unordered_map<Id, double> transitionProbabilities;
    double cumulativeProbability = 0.0;

    for (const auto outEdgeId : outgoingEdges) {
      if (forbiddenTurns.contains(outEdgeId)) {
        spdlog::trace("Forbidden turn from street {} to street {}. Skipping.",
                      pAgent->streetId().value_or(0),
                      outEdgeId);
        continue;
      }

      auto const& streetOut{this->graph().edge(outEdgeId)};

      // Check if this is a valid path target for non-random agents
      bool bIsPathTarget = false;
      bIsPathTarget =
          std::find(pathTargets.cbegin(), pathTargets.cend(), streetOut.target()) !=
          pathTargets.cend();

      if (!this->m_errorProbability.has_value() && !bIsPathTarget) {
        continue;
      }

      double probability = 1.0;
      //std::exp(-0.5 * pStreetOut->maxSpeed() / (this->m_speedFunction(*pStreetOut)));

      // Apply error probability for non-random agents
      if (this->m_errorProbability.has_value()) {
        probability *=
            (bIsPathTarget
                 ? (1. - this->m_errorProbability.value())
                 : this->m_errorProbability.value() /
                       static_cast<double>(outgoingEdges.size() - pathTargets.size()));
      }

      // Handle U-turns
      if (previousNodeId.has_value() && streetOut.target() == previousNodeId.value()) {
        if (pNode->isRoundabout()) {
          probability *= m_uturnPenaltyFactor;  // Penalize U-turns in roundabouts
        } else if (!bIsPathTarget) {
          continue;  // No U-turns allowed in non-roundabout nodes
        }
      }

      transitionProbabilities.emplace(streetOut.id(), probability);
      cumulativeProbability += probability;
    }

    return m_extractStreet(transitionProbabilities, cumulativeProbability);
  }

  void FirstOrderDynamics::m_evolveStreet(Street* pStreet) {
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
        this->m_removeAgent<false>(pStreet->dequeueMovingAgent());
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
      std::vector<size_t> validLanes;
      auto const& mapping = pStreet->laneMapping();
      for (size_t laneIndex = 0; laneIndex < mapping.size(); ++laneIndex) {
        switch (direction) {
          case Direction::ANY:
            validLanes.push_back(laneIndex);
            break;
          case Direction::UTURN:
          case Direction::LEFT:
            if (mapping[laneIndex] == Direction::LEFT ||
                mapping[laneIndex] == Direction::LEFTANDSTRAIGHT) {
              validLanes.push_back(laneIndex);
            }
            break;
          case Direction::STRAIGHT:
            if (mapping[laneIndex] == Direction::STRAIGHT ||
                mapping[laneIndex] == Direction::LEFTANDSTRAIGHT ||
                mapping[laneIndex] == Direction::RIGHTANDSTRAIGHT) {
              validLanes.push_back(laneIndex);
            }
            break;
          case Direction::RIGHT:
            if (mapping[laneIndex] == Direction::RIGHT ||
                mapping[laneIndex] == Direction::RIGHTANDSTRAIGHT) {
              validLanes.push_back(laneIndex);
            }
            break;
          default:
            validLanes.push_back(laneIndex);
        }
      }
      std::vector<double> weights;
      auto itValidLane = validLanes.cbegin();
      for (std::size_t laneIndex = 0;
           laneIndex < static_cast<std::size_t>(pStreet->nLanes());
           ++laneIndex) {
        if (itValidLane != validLanes.cend() && laneIndex == *itValidLane) {
          weights.push_back(1. / (pStreet->queue(*itValidLane).size() + 1));
          ++itValidLane;
        } else {
          weights.push_back(0.);
        }
      }
      // If all weights are the same, make the last 0
      if (std::all_of(weights.begin(), weights.end(), [&](double w) {
            return std::abs(w - weights.front()) < std::numeric_limits<double>::epsilon();
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
    auto const& transportCapacity{pStreet->transportCapacity()};
    std::uniform_real_distribution<double> uniformDist{0., 1.};
    for (auto queueIndex = 0; queueIndex < nLanes; ++queueIndex) {
      if (pStreet->queue(queueIndex).empty()) {
        continue;
      }
      if (uniformDist(this->m_generator) > transportCapacity) {
        spdlog::trace("Skipping due to transport capacity {} < random {}",
                      transportCapacity,
                      uniformDist(this->m_generator));
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
          this->m_removeAgent<false>(pStreet->dequeue(queueIndex, this->time_step()));
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
                        .turnDirection(
                            this->graph().edge(pAgentTemp2->streetId().value()).angle())};
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
            this->m_removeAgent<true>(pStreet->dequeue(queueIndex, this->time_step()));
        if (m_reinsertAgents) {
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
            spdlog::trace("Incremented turn count for {} -> {}: {}",
                          *(pAgent->streetId()),
                          nextStreet->id(),
                          m_turnCounts[*(pAgent->streetId())][nextStreet->id()]);
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
            spdlog::trace("Incremented turn count for {} -> {}: {}",
                          *(pAgentTemp->streetId()),
                          nextStreet->id(),
                          m_turnCounts[*(pAgentTemp->streetId())][nextStreet->id()]);
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
        spdlog::debug("No next street id, generating a new one");
        auto const nextStreetId{this->m_nextStreetId(pAgent, pSourceNode)};
        if (!nextStreetId.has_value()) {
          spdlog::debug(
              "No next street found for agent {} at node {}", *pAgent, pSourceNode->id());
          itAgent = m_agents.erase(itAgent);
          auto pAgentRemoved{this->m_removeAgent<false>(std::move(*itAgent))};
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

  void FirstOrderDynamics::prepareNetwork(bool const bAdjustNodeCapacities,
                                          bool const bAutoMapStreetLanes,
                                          bool const bAutoAssignRoadPriorities,
                                          bool const bAutoInitTrafficLights) {
    if (bAdjustNodeCapacities) {
      m_graph->adjustNodeCapacities();
    }
    if (bAutoMapStreetLanes) {
      m_graph->autoMapStreetLanes();
    }
    if (bAutoAssignRoadPriorities) {
      m_graph->autoAssignRoadPriorities();
    }
    if (bAutoInitTrafficLights) {
      m_graph->autoInitTrafficLights();
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
    if (!std::filesystem::exists(fileName)) {
      throw std::invalid_argument(std::format("File {} does not exist", fileName));
    }
    csv::CSVFormat format;
    format.delimiter(separator);
    csv::CSVReader reader(fileName, format);

    auto const& colNames = reader.get_col_names();

    AgentInsertionMethod csvtype{AgentInsertionMethod::RANDOM};
    for (auto const& colName : colNames) {
      if (colName == "node_id") {
        csvtype = AgentInsertionMethod::RANDOM_ODS;
        break;
      } else if (colName == "destination_id") {
        csvtype = AgentInsertionMethod::ODS;
        break;
      } else if (colName == "destinations") {
        csvtype = AgentInsertionMethod::CONDITIONAL_RANDOM_ODS;
        break;
      }
    }

    switch (csvtype) {
      case AgentInsertionMethod::RANDOM_ODS: {
        bool bCompatFormat{false};
        for (auto const& colName : colNames) {
          if (colName == "o_prob" || colName == "d_prob") {
            bCompatFormat = true;
            break;
          }
        }
        std::unordered_map<Id, double> originNodes;
        std::unordered_map<Id, double> destinationNodes;
        if (bCompatFormat) {
          spdlog::info("Importing ODs from CSV with RANDOM_ODS method (compat format).");
          for (auto const& row : reader) {
            auto const nodeId = row["node_id"].get<Id>();
            // try to get o_prob as double
            try {
              auto oProb = row["o_prob"].get<double>();
              originNodes.emplace(nodeId, oProb);
            } catch (...) {
              // Do nothing, the node will be skipped as origin
            }
            try {
              auto dProb = row["d_prob"].get<double>();
              destinationNodes.emplace(nodeId, dProb);
            } catch (...) {
              // Do nothing, the node will be skipped as destination
            }
          }
        } else {
          spdlog::info(
              "Importing ODs from CSV with RANDOM_ODS method (extended format).");
          for (auto const& row : reader) {
            auto const nodeId = row["node_id"].get<Id>();
            auto const type = row["type"].get<std::string>();
            auto const weight = row["weight"].get<double>();
            if (type == "O") {
              originNodes[nodeId] = weight;
            } else if (type == "D") {
              destinationNodes[nodeId] = weight;
            } else {
              spdlog::warn("Unknown type '{}' for node {} in CSV. Skipping this row.",
                           type,
                           nodeId);
            }
          }
        }
        spdlog::info("Imported {} origin nodes and {} destination nodes from CSV.",
                     originNodes.size(),
                     destinationNodes.size());
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
      case AgentInsertionMethod::CONDITIONAL_RANDOM_ODS: {
        spdlog::info("Importing ODs from CSV with CONDITIONAL_RANDOM_ODS method.");
        std::unordered_map<Id, std::tuple<double, std::vector<std::tuple<Id, double>>>>
            conditionalODs;
        for (auto const& row : reader) {
          auto const originId = row["origin_id"].get<Id>();
          auto const weight = row["weight"].get<double>();
          // Get destination nodes as id:weight,id:weight,...
          auto const destinationsStr = row["destinations"].get<std::string>();
          if (destinationsStr.empty()) {
            spdlog::warn("Empty destinations for origin {} in CSV. Skipping this row.",
                         originId);
            continue;
          }
          conditionalODs[originId] =
              std::make_tuple(weight, std::vector<std::tuple<Id, double>>());
          std::istringstream ss(destinationsStr);
          std::string destinationPair;
          while (std::getline(ss, destinationPair, ',')) {
            auto const colonPos = destinationPair.find(':');
            if (colonPos == std::string::npos) {
              spdlog::warn(
                  "Invalid destination pair '{}' for origin {} in CSV. Skipping this "
                  "pair.",
                  destinationPair,
                  originId);
              continue;
            }
            auto const destinationId =
                static_cast<Id>(std::stoul(destinationPair.substr(0, colonPos)));
            auto const destinationWeight =
                std::stod(destinationPair.substr(colonPos + 1));
            std::get<1>(conditionalODs.at(originId))
                .emplace_back(destinationId, destinationWeight);
          }
        }
        this->setConditionalODs(std::move(conditionalODs));
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
      throw std::runtime_error(
          std::format("Turn counts have already been initialized (current size: {}).",
                      m_turnCounts.size()));
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
    spdlog::debug(
        "FirstOrderDynamics::saveData is now a compatibility hook. interval={}s, "
        "avg_stats={}, street_data={}, travel_data={}, agent_data={}",
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
  void FirstOrderDynamics::setConditionalODs(
      std::unordered_map<Id,
                         std::tuple<double, std::vector<std::tuple<Id, double>>>> const&
          conditionalODs) {
    m_originToDestinations.clear();
    m_originToDestinations.reserve(conditionalODs.size());
    std::unordered_map<Id, double> originWeights;
    for (auto const& [originId, destinations] : conditionalODs) {
      originWeights[originId] = std::get<0>(destinations);
      auto sumDestinationWeights = std::accumulate(
          std::get<1>(destinations).begin(),
          std::get<1>(destinations).end(),
          0.,
          [](double sum, auto const& tuple) { return sum + std::get<1>(tuple); });
      if (sumDestinationWeights <= 0.) {
        throw std::invalid_argument(std::format(
            "The sum of the destination weights ({}) for origin {} must be positive",
            sumDestinationWeights,
            originId));
      }
      m_originToDestinations[originId].reserve(std::get<1>(destinations).size());
      for (auto const& [destinationId, weight] : std::get<1>(destinations)) {
        if (!this->itineraries().contains(destinationId)) {
          this->addItinerary(destinationId, destinationId);
        }
        m_originToDestinations.at(originId).emplace_back(destinationId,
                                                         weight / sumDestinationWeights);
      }
    }
    this->setOriginNodes(std::move(originWeights));
  }

  void FirstOrderDynamics::updatePaths() {
    spdlog::debug("Init updating paths...");
    tbb::concurrent_vector<Id> emptyItineraries;
    tbb::parallel_for_each(
        this->itineraries().cbegin(),
        this->itineraries().cend(),
        [this, &emptyItineraries](auto const& pair) -> void {
          auto const& pItinerary{pair.second};
          this->m_updatePath(pItinerary);
          if (pItinerary->empty()) {
            if (!this->m_updatepathsThrowOnEmpty) {
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
      auto const strIds = std::accumulate(
          emptyItineraries.begin(),
          emptyItineraries.end(),
          std::string{},
          [](std::string const& a, Id const& b) {
            return a.empty() ? std::to_string(b) : a + ", " + std::to_string(b);
          });
      spdlog::warn("Removing {} itineraries with no valid path from the dynamics ({}).",
                   emptyItineraries.size(),
                   strIds);
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
        for (auto& [origin, destinations] : m_originToDestinations) {
          std::erase_if(destinations, [destination](auto const& tuple) {
            return std::get<0>(tuple) == destination;
          });
        }
        // Remove all origins in m_originToDestinations that have no destinations left
        std::erase_if(m_originToDestinations, [this](auto const& pair) {
          bool const bErase{pair.second.empty()};
          if (bErase) {
            std::erase_if(m_originNodes, [&pair](auto const& tuple) {
              return std::get<0>(tuple) == pair.first;
            });
          }
          return bErase;
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
      case AgentInsertionMethod::CONDITIONAL_RANDOM_ODS:
        this->m_addAgentsConditionalRandomODs(nAgents);
        break;
      case AgentInsertionMethod::UNIFORM:
        this->addAgentsUniformly(nAgents);
        break;
      default:
        throw std::runtime_error(
            "Cannot add agents without a valid insertion methods. Possible values are "
            "\"RANDOM\", \"ODS\", \"RANDOM_ODS\" and \"UNIFORM\"");
    }
  }

  void FirstOrderDynamics::addItinerary(std::shared_ptr<Itinerary> itinerary) {
    if (m_itineraries.contains(itinerary->id())) {
      throw std::invalid_argument(
          std::format("Itinerary with id {} already exists.", itinerary->id()));
    }
    m_itineraries.emplace(itinerary->id(), std::move(itinerary));
  }

  StepDataResult FirstOrderDynamics::evolve(StepDataRequest const& dataRequest) {
    StepDataResult stepData;
    stepData.timeStep = this->time_step();
    auto const n_threads{std::max<std::size_t>(1, this->concurrency())};
    std::atomic<double> mean_speed{0.}, mean_density{0.}, mean_traveltime{0.},
        mean_queue_length{0.};
    std::atomic<double> std_speed{0.}, std_density{0.};
    std::atomic<std::size_t> nValidEdges{0};
    bool const bComputeStats = dataRequest.saveAverageStats ||
                               dataRequest.saveStreetData || dataRequest.saveTravelData ||
                               dataRequest.saveAgentData;
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
                    // Walk every phase and register each (street, direction) pair once.
                    for (auto const& phase : tl.phases()) {
                      for (auto const& [streetId, dirSet] : phase.greenSet()) {
                        for (auto const dir : dirSet) {
                          m_queuesAtTrafficLights[streetId].emplace(dir, 0.);
                        }
                      }
                    }
                  }
                  for (auto& [direction, value] : m_queuesAtTrafficLights.at(inEdgeId)) {
                    value += pStreet->nExitingAgents(direction, true);
                  }
                }
                m_evolveStreet(pStreet);
                if (bComputeStats) {
                  auto const& density{pStreet->density<false>() * 1e3};
                  auto const& queueLength{pStreet->nExitingAgents()};

                  auto const speedMeasure = pStreet->meanSpeed<true>();
                  if (speedMeasure.is_valid) {
                    auto const speed = speedMeasure.mean * 3.6;  // to kph
                    auto const speed_std = speedMeasure.std * 3.6;
                    if (dataRequest.saveAverageStats) {
                      mean_speed.fetch_add(speed, std::memory_order_relaxed);
                      std_speed.fetch_add(speed * speed + speed_std * speed_std,
                                          std::memory_order_relaxed);
                      mean_traveltime.fetch_add(pStreet->length() / speedMeasure.mean,
                                                std::memory_order_relaxed);
                      ++nValidEdges;
                    }
                  }
                  if (dataRequest.saveAverageStats) {
                    mean_density.fetch_add(density, std::memory_order_relaxed);
                    std_density.fetch_add(density * density, std::memory_order_relaxed);
                    mean_queue_length.fetch_add(queueLength, std::memory_order_relaxed);
                  }

                  if (dataRequest.saveStreetData) {
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
      if (dataRequest.saveStreetData && !streetDataRecords.empty()) {
        stepData.streetData = std::move(streetDataRecords);
      }

      if (dataRequest.saveTravelData && !m_travelDTs.empty()) {
        stepData.travelData = std::move(m_travelDTs);
        m_travelDTs.clear();
      }

      if (dataRequest.saveAgentData) {
        stepData.agentData = Street::agentData();
      }

      if (dataRequest.saveAverageStats) {
        AverageStatsRecord averageStats;
        averageStats.nValidEdges = nValidEdges.load();
        auto const edgeCount = static_cast<double>(numEdges);
        if (averageStats.nValidEdges > 0) {
          averageStats.meanSpeed = mean_speed.load() / averageStats.nValidEdges;
          averageStats.stdSpeed =
              std::sqrt(std::max(0.0,
                                 std_speed.load() / averageStats.nValidEdges -
                                     averageStats.meanSpeed * averageStats.meanSpeed));
          averageStats.meanDensity = mean_density.load() / edgeCount;
          averageStats.stdDensity = std::sqrt(
              std::max(0.0,
                       std_density.load() / edgeCount -
                           averageStats.meanDensity * averageStats.meanDensity));
          averageStats.meanTravelTime = mean_traveltime.load() / averageStats.nValidEdges;
          averageStats.meanQueueLength = mean_queue_length.load() / edgeCount;
        }
        stepData.averageStats = averageStats;
      }
    }
    if (dataRequest.saveTurnCounts) {
      stepData.turnCounts = m_turnCounts;
      this->resetTurnCounts();
    }

    Dynamics<RoadNetwork>::m_evolve();
    return stepData;
  }

  void FirstOrderDynamics::m_trafficlightSingleTailOptimizer(
      double const beta, std::optional<std::ofstream>& logStream) {
    if (beta < 0. || beta > 1.) {
      throw std::invalid_argument(
          std::format("The beta parameter ({}) must be in [0, 1]", beta));
    }
    if (logStream.has_value()) {
      *logStream << std::format(
          "Init Traffic Lights optimisation (SINGLE TAIL) — time {} — beta {:.2f}\n",
          this->time_step(),
          beta);
    }

    for (auto const& [nodeId, pNode] : this->graph().nodes()) {
      if (!pNode->isTrafficLight())
        continue;
      auto& tl = dynamic_cast<TrafficLight&>(*pNode);
      if (tl.phases().empty())
        continue;

      auto const nPhases = tl.phases().size();
      auto const currentCycle = tl.cycleTime();
      if (currentCycle == 0)
        continue;

      // ── Step 1: compute demand per phase ───────────────────────────────
      // Demand for phase i = sum of queue lengths of every (street, dir)
      // pair that appears in that phase's green set.
      std::vector<double> demand(nPhases, 0.);
      for (std::size_t i = 0; i < nPhases; ++i) {
        for (auto const& [streetId, dirSet] : tl.phases()[i].greenSet()) {
          auto const queueIt = m_queuesAtTrafficLights.find(streetId);
          if (queueIt == m_queuesAtTrafficLights.end())
            continue;
          for (auto const& [dir, count] : queueIt->second) {
            if (dirSet.contains(dir) || dirSet.contains(Direction::ANY)) {
              demand[i] += count;
            }
          }
        }
      }

      double const totalDemand = std::accumulate(demand.cbegin(), demand.cend(), 0.);
      if (totalDemand == 0.) {
        spdlog::debug("TrafficLight {}: no queue data — skipping optimisation.", nodeId);
        continue;
      }

      // ── Step 2: normalise demand → demand ratio per phase ──────────────
      std::vector<double> demandRatio(nPhases);
      for (std::size_t i = 0; i < nPhases; ++i)
        demandRatio[i] = demand[i] / totalDemand;

      // ── Step 3: baseline duration ratio (from stored defaults) ─────────
      // The optimiser always blends against the originally configured timing
      // so that repeated optimisation calls do not drift away from the
      // designed baseline.
      auto const& defaults = tl.defaultPhases();
      double const defaultCycle = static_cast<double>(std::accumulate(
          defaults.cbegin(),
          defaults.cend(),
          Delay{0},
          [](Delay s, TrafficLightPhase const& p) { return s + p.duration(); }));

      std::vector<double> baseRatio(nPhases);
      for (std::size_t i = 0; i < nPhases; ++i)
        baseRatio[i] = (defaultCycle > 0.)
                           ? static_cast<double>(defaults[i].duration()) / defaultCycle
                           : 1. / static_cast<double>(nPhases);

      // ── Step 4: blend ──────────────────────────────────────────────────
      //   targetRatio[i] = beta * demand[i] + (1 - beta) * base[i]
      std::vector<double> targetRatio(nPhases);
      for (std::size_t i = 0; i < nPhases; ++i)
        targetRatio[i] = beta * demandRatio[i] + (1. - beta) * baseRatio[i];

      // ── Step 5: convert to integer durations summing to currentCycle ───
      std::vector<Delay> newDurations(nPhases);
      std::vector<double> fractionals(nPhases);
      Delay allocatedTotal{0};

      for (std::size_t i = 0; i < nPhases; ++i) {
        double const exact = targetRatio[i] * static_cast<double>(currentCycle);
        newDurations[i] = static_cast<Delay>(std::floor(exact));
        fractionals[i] = exact - static_cast<double>(newDurations[i]);
        allocatedTotal += newDurations[i];
      }

      // Distribute remaining ticks to phases with the largest fractional parts.
      Delay remainder = currentCycle - allocatedTotal;
      std::vector<std::size_t> order(nPhases);
      std::iota(order.begin(), order.end(), 0u);
      std::sort(order.begin(), order.end(), [&](auto a, auto b) {
        return fractionals[a] > fractionals[b];
      });
      for (Delay r = 0; r < remainder; ++r)
        ++newDurations[order[r % nPhases]];

      // Guarantee every phase gets at least 1 tick when feasible, while
      // preserving the invariant that the total duration equals currentCycle.
      if (currentCycle >= static_cast<Delay>(nPhases)) {
        Delay extraTicksNeeded{0};
        for (auto& d : newDurations) {
          if (d < Delay{1}) {
            d = Delay{1};
            ++extraTicksNeeded;
          }
        }
        if (extraTicksNeeded > 0) {
          std::vector<std::size_t> donors(nPhases);
          std::iota(donors.begin(), donors.end(), 0u);
          std::sort(donors.begin(), donors.end(), [&](auto a, auto b) {
            return newDurations[a] > newDurations[b];
          });
          for (auto idx : donors) {
            while (extraTicksNeeded > 0 && newDurations[idx] > Delay{1}) {
              --newDurations[idx];
              --extraTicksNeeded;
            }
            if (extraTicksNeeded == 0) {
              break;
            }
          }
        }
      }

      // ── Step 6: apply durations in-place ───────────────────────────────
      for (std::size_t i = 0; i < nPhases; ++i)
        tl.phase(i).setDuration(newDurations[i]);

      if (logStream.has_value()) {
        *logStream << std::format("\nNew phases for {}", tl);
      }
      spdlog::debug("TrafficLight {}: optimised {} phases (cycle={} ticks).",
                    nodeId,
                    nPhases,
                    currentCycle);
    }

    if (logStream.has_value()) {
      *logStream << std::format("End Traffic Lights optimisation — time {}\n",
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
          density += pStreet->density<true>();
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
          auto const travelTimeTicks =
              static_cast<Delay>(std::round(pStreet->estimatedTravelTime()));
          tl.advanceBy(travelTimeTicks);
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

  Measurement<double> FirstOrderDynamics::streetMeanFlow() const {
    std::vector<double> flows;
    flows.reserve(this->graph().nEdges());
    for (const auto& [streetId, pStreet] : this->graph().edges()) {
      auto const speedMeasure = pStreet->meanSpeed<true>();
      if (speedMeasure.is_valid) {
        flows.push_back(pStreet->density<false>() * speedMeasure.mean);
      }
    }
    return Measurement<double>(flows);
  }

  Measurement<double> FirstOrderDynamics::streetMeanFlow(double threshold,
                                                         bool above) const {
    std::vector<double> flows;
    flows.reserve(this->graph().nEdges());
    for (const auto& [streetId, pStreet] : this->graph().edges()) {
      auto const speedMeasure = pStreet->meanSpeed<true>();
      if (!speedMeasure.is_valid) {
        continue;
      }
      if (above && (pStreet->density<true>() > threshold)) {
        flows.push_back(pStreet->density<false>() * speedMeasure.mean);
      } else if (!above && (pStreet->density<true>() < threshold)) {
        flows.push_back(pStreet->density<false>() * speedMeasure.mean);
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

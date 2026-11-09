#include "FirstOrderDynamics.hpp"

#include <charconv>

#include <csv.hpp>
#include <simdjson.h>
#include <SQLiteCpp/SQLiteCpp.h>
#include <spdlog/spdlog.h>

namespace dsf::mobility {
  FirstOrderDynamics::FirstOrderDynamics(RoadNetwork&& graph,
                                         std::optional<unsigned int> seed,
                                         DynamicsMode const mode)
      : Dynamics<RoadNetwork>(std::move(graph), seed), m_mode{mode} {
    // Set defaults for speed function
    this->setSpeedFunction(SpeedFunction::LINEAR, 0.8);
    for (auto const& [nodeId, pNode] : this->graph().nodes()) {
      m_nodeIndices.push_back(nodeId);
    }
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
      if (colName == "id") {
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
        std::unordered_map<Id, double> origins;
        std::unordered_map<Id, double> destinations;
        if (bCompatFormat) {
          spdlog::info("Importing ODs from CSV with RANDOM_ODS method (compat format).");
          for (auto const& row : reader) {
            auto const id = row["id"].get<Id>();
            // try to get o_prob as double
            try {
              auto oProb = row["o_prob"].get<double>();
              origins.emplace(id, oProb);
            } catch (...) {
              // Do nothing, the id will be skipped as origin
            }
            try {
              auto dProb = row["d_prob"].get<double>();
              destinations.emplace(id, dProb);
            } catch (...) {
              // Do nothing, the id will be skipped as destination
            }
          }
        } else {
          spdlog::info(
              "Importing ODs from CSV with RANDOM_ODS method (extended format).");
          for (auto const& row : reader) {
            auto const id = row["id"].get<Id>();
            auto const type = row["type"].get<std::string>();
            auto const weight = row["weight"].get<double>();
            if (type == "O") {
              origins[id] = weight;
            } else if (type == "D") {
              destinations[id] = weight;
            } else {
              spdlog::warn(
                  "Unknown type '{}' for id {} in CSV. Skipping this row.", type, id);
            }
          }
        }
        spdlog::info("Imported {} origins and {} destinations from CSV.",
                     origins.size(),
                     destinations.size());
        this->setOrigins(std::move(origins));
        this->setDestinations(std::move(destinations));
        break;
      }
      case AgentInsertionMethod::ODS: {
        spdlog::info("Importing ODs from CSV with RANDOM_OD_PAIRS method.");
        std::vector<ODEntry> ODs;
        for (auto const& row : reader) {
          auto const originId = row["origin_id"].get<Id>();
          auto const destinationId = row["destination_id"].get<Id>();
          auto const weight = row["weight"].get<double>();
          ODs.push_back({originId, destinationId, weight});
        }
        this->setODs(std::move(ODs));
        break;
      }
      case AgentInsertionMethod::CONDITIONAL_RANDOM_ODS: {
        spdlog::info("Importing ODs from CSV with CONDITIONAL_RANDOM_ODS method.");
        std::unordered_map<Id, ConditionalOrigin> conditionalODs;
        for (auto const& row : reader) {
          auto const originId = row["origin_id"].get<Id>();
          auto const weight = row["weight"].get<double>();
          // Get destinations as id:weight,id:weight,...
          auto const destinationsStr = row["destinations"].get<std::string>();
          if (destinationsStr.empty()) {
            spdlog::warn("Empty destinations for origin {} in CSV. Skipping this row.",
                         originId);
            continue;
          }
          conditionalODs[originId] =
              ConditionalOrigin{.weight = weight, .destinations = {}};
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
            conditionalODs.at(originId).destinations.push_back(
                {destinationId, destinationWeight});
          }
        }
        this->setConditionalODs(std::move(conditionalODs));
        break;
      }
      default:
        throw std::runtime_error(
            "Could not determine the CSV type based on column names. Expected columns: "
            "'id' for RANDOM_ODS or 'origin_id' and 'destination_id' for "
            "RANDOM_OD_PAIRS.");
    }
    spdlog::info("Finished importing ODs from CSV.");
  }

  void FirstOrderDynamics::m_dropPendingAgent(std::unique_ptr<Agent>& pAgent) {
    spdlog::debug("Dropping {}: there is no route to its destination", *pAgent);
    pAgent.reset();
    --m_nAgents;
    ++m_nKilledAgents;
  }
  void FirstOrderDynamics::m_ensureRouter() {
    // Streets and nodes are only added, so the counts catch the additions; every other
    // change which affects routing bumps the routing revision
    if (!m_router.isBuilt() || m_routerRevision != Road::routingRevision() ||
        m_router.nEdges() != this->graph().nEdges() ||
        m_routerNodes != this->graph().nNodes()) {
      m_rebuildRouter();
    } else if (!m_bDestinationsChecked) {
      m_checkDestinations();
    }
  }
  double FirstOrderDynamics::m_distanceFactor(Agent const& agent) const {
    if (m_router.distanceCost() == 0.) {
      return 0.;
    }
    // Each agent has its own distance cost, drawn from the seed and its id: it draws nothing
    // from the engines, and stays the same when the agent plans again
    return Router::distanceFactor(this->seed() + 0x9E3779B97F4A7C15ULL * agent.id(),
                                  m_router.distanceCost(),
                                  m_router.distanceSigma());
  }
  std::vector<Id> FirstOrderDynamics::m_planRoute(Agent const& agent,
                                                  Id const fromStreetId) const {
    // Agents drive their destination street, and arrive at its end
    return m_router.route(
        fromStreetId,
        *agent.destination(),
        agent.isIntelligent() ? RouteWeights::CURRENT : RouteWeights::FREE_FLOW,
        m_distanceFactor(agent));
  }
  std::vector<Id> FirstOrderDynamics::m_planRouteFromNode(Agent const& agent,
                                                          Id const fromNodeId) const {
    return m_router.routeFromNode(
        fromNodeId,
        *agent.destination(),
        agent.isIntelligent() ? RouteWeights::CURRENT : RouteWeights::FREE_FLOW,
        m_distanceFactor(agent));
  }
  void FirstOrderDynamics::m_setPath(Agent& agent, std::span<Id const> route) {
    // The agent keeps its destination, the last street of the route, by itself
    if (route.empty()) {
      agent.dropPath();
      return;
    }
    agent.setPath(route.first(route.size() - 1));
  }
  void FirstOrderDynamics::m_assignIntelligence(std::unique_ptr<Agent> const& pAgent) {
    if (m_intelligentFraction <= 0. || pAgent->isRandom()) {
      return;
    }
    // Skip the draw when every agent is intelligent, to leave the generator untouched
    std::bernoulli_distribution intelligenceDist{m_intelligentFraction};
    pAgent->setIntelligent(m_intelligentFraction >= 1. ||
                           intelligenceDist(this->m_generator));
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
      m_nKilledAgents += nStagnantAgents;
    }
    if (m_ODs.size() == 1) {
      auto [originId, destinationId, weight] = m_ODs.at(0);
      if (!m_destinationStreets.contains(destinationId)) {
        spdlog::warn("Skipping ODS insertion: destination {} not found", destinationId);
        return;
      }
      for (std::size_t i = 0; i < nAgents; ++i) {
        this->addAgent(destinationId, originId);
      }
      return;
    }
    if (m_ODCumulativeWeights.size() != m_ODs.size()) {
      m_ODCumulativeWeights.clear();
      m_ODCumulativeWeights.reserve(m_ODs.size());
      double cumulativeWeight{0.};
      for (auto const& od : m_ODs) {
        cumulativeWeight += od.weight;
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
          std::ranges::lower_bound(m_ODCumulativeWeights, randValue) -
          m_ODCumulativeWeights.cbegin());
      if (selectedOdIdx >= m_ODs.size()) {
        selectedOdIdx = m_ODs.size() - 1;
      }

      auto const& [originId, destinationId, weight] = m_ODs[selectedOdIdx];
      if (!m_destinationStreets.contains(destinationId)) {
        spdlog::warn("Skipping ODS insertion: destination {} not found", destinationId);
        continue;
      }
      this->addAgent(destinationId, originId);
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
      m_nKilledAgents += nStagnantAgents;
    }
    auto const& nSources{m_origins.size()};
    auto const& nDestinations{m_destinations.size()};
    spdlog::debug("Init addAgentsRandomly for {} agents from {} streets to {} streets.",
                  nAgents,
                  nSources,
                  nDestinations);
    if (nSources == 1 && nDestinations == 1 &&
        m_origins.at(0).id == m_destinations.at(0).id) {
      throw std::invalid_argument(
          std::format("The only origin street {} is also the only destination street.",
                      m_origins.at(0).id));
    }
    std::uniform_real_distribution<double> uniformDist{0., 1.};
    auto const originCumulativeWeights{m_cumulativeWeights(m_origins)};
    auto const destinationCumulativeWeights{m_cumulativeWeights(m_destinations)};
    spdlog::debug("Adding {} agents at time {}.", nAgents, this->time_step());
    while (nAgents--) {
      std::optional<Id> srcId{std::nullopt}, dstId{std::nullopt};

      // Select source using weighted random selection. Origins and destinations are
      // street ids, so the fallback must stay within them: picking a random *node*
      // here would hand a node id to addAgent's source-street parameter.
      if (nSources == 1) {
        srcId = m_origins.at(0).id;
      } else {
        srcId = m_pickWeighted(
            m_origins, originCumulativeWeights, uniformDist(this->m_generator));
      }

      // Select destination using weighted random selection
      if (nDestinations == 1) {
        dstId = m_destinations.at(0).id;
      } else {
        dstId = m_pickWeighted(
            m_destinations, destinationCumulativeWeights, uniformDist(this->m_generator));
      }

      if (!m_destinationStreets.contains(*dstId)) {
        spdlog::error("Destination {} not found. Skipping agent.", *dstId);
        continue;
      }

      // Routes are planned at departure, which drops the agents that cannot reach their
      // destination: only trips to the origin itself are skipped here
      if (*srcId == *dstId) {
        spdlog::debug("Origin {} is also the destination. Skipping agent.", *srcId);
        continue;
      }

      this->addAgent(*dstId, *srcId);
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
      m_nKilledAgents += nStagnantAgents;
    }
    if (m_origins.empty()) {
      throw std::runtime_error(
          "FirstOrderDynamics::m_addAgentsConditionalRandomODs: Origins must be set");
    }
    std::uniform_real_distribution<double> uniformDist{0., 1.};
    auto const originCumulativeWeights{m_cumulativeWeights(m_origins)};
    spdlog::debug("Adding {} agents at time {}.", nAgents, this->time_step());
    while (nAgents--) {
      // Select origin using weighted random selection
      auto const originId{m_pickWeighted(
          m_origins, originCumulativeWeights, uniformDist(this->m_generator))};
      // Select destination conditionally based on the selected origin
      auto originToDestIt = m_originToDestinations.find(originId);
      if (originToDestIt == m_originToDestinations.end() ||
          originToDestIt->second.empty()) {
        throw std::runtime_error(std::format(
            "No destinations found for origin {} in conditional random OD insertion.",
            originId));
      }
      const auto& destinations = originToDestIt->second;
      auto randValue = uniformDist(this->m_generator);
      Id destinationId{
          destinations.back().id};  // fallback to last for numerical stability
      for (const auto& [id, weight] : destinations) {
        if (randValue < weight) {
          destinationId = id;
          break;
        }
        randValue -= weight;
      }
      if (!m_destinationStreets.contains(destinationId)) {
        spdlog::error("Destination {} not found. Skipping agent.", destinationId);
        continue;
      }
      // Routes are planned at departure, which drops the agents that cannot reach their
      // destination: only trips to the origin itself are skipped here
      if (originId == destinationId) {
        spdlog::debug("Origin {} is also the destination. Skipping agent.", originId);
        continue;
      }
      this->addAgent(destinationId, originId);
    }
  }

  Street const* FirstOrderDynamics::m_currentStreet(const std::unique_ptr<Agent>& pAgent,
                                                    Street const* pHint) const {
    auto const currentStreetIdOpt{pAgent->streetId()};
    if (!currentStreetIdOpt.has_value()) {
      return nullptr;
    }
    if (pHint != nullptr && pHint->id() == *currentStreetIdOpt) {
      return pHint;
    }
    return &this->graph().edge(*currentStreetIdOpt);
  }

  Street const* FirstOrderDynamics::m_nextRandomStreet(
      const std::unique_ptr<Agent>& pAgent,
      RoadJunction const* pNode,
      Street const* pCurrentStreet,
      std::mt19937_64& rng) {
    spdlog::trace("Computing m_nextRandomStreet for {}", *pAgent);
    auto const& outgoingEdges = pNode->outgoingEdges();

    // Calculate transition probabilities for all valid outgoing edges. The buffer is reused
    // across calls (one per thread, as nodes evolve in parallel) to avoid an allocation per
    // routing decision
    thread_local std::vector<std::pair<Street const*, double>> transitionProbabilities;
    transitionProbabilities.clear();
    double cumulativeProbability = 0.0;

    static std::set<Id> const noForbiddenTurns;
    auto const* pForbiddenTurns{&noForbiddenTurns};
    Node const* pPreviousNode{nullptr};
    auto const currentStreetIdOpt{pAgent->streetId()};
    auto const* pStreetCurrent{m_currentStreet(pAgent, pCurrentStreet)};
    if (pStreetCurrent != nullptr) {
      pForbiddenTurns = &pStreetCurrent->forbiddenTurns();
      pPreviousNode = pStreetCurrent->source();
    }

    auto const itRow{currentStreetIdOpt.has_value()
                         ? m_transitionMatrix.find(*currentStreetIdOpt)
                         : m_transitionMatrix.cend()};

    auto const collectCandidates = [&](bool const bUseMatrix) -> void {
      transitionProbabilities.clear();
      cumulativeProbability = 0.0;
      for (auto const* pStreetOut : outgoingEdges) {
        auto const outEdgeId{pStreetOut->id()};
        if (pForbiddenTurns->contains(outEdgeId)) {
          spdlog::trace("Forbidden turn from street {} to street {}. Skipping.",
                        pAgent->streetId().value_or(0),
                        outEdgeId);
          continue;
        }
        auto const& streetOut{*pStreetOut};

        // Handle U-turns
        if (pPreviousNode != nullptr && streetOut.target() == pPreviousNode) {
          continue;
        }

        double probability = 1.0;
        if (bUseMatrix) {
          auto const itWeight{itRow->second.find(streetOut.id())};
          if (itWeight == itRow->second.cend()) {
            continue;
          }
          probability = itWeight->second;
        }

        transitionProbabilities.emplace_back(pStreetOut, probability);
        cumulativeProbability += probability;
      }
    };

    bool bUseMatrix{itRow != m_transitionMatrix.cend()};
    collectCandidates(bUseMatrix);
    if (bUseMatrix && transitionProbabilities.empty()) {
      // Every street listed for this row has been filtered out: fall back to the uniform
      // behaviour rather than terminating every agent passing by.
      spdlog::debug(
          "No usable transition-matrix entry for street {} at {}. Falling back to a "
          "uniform choice.",
          *currentStreetIdOpt,
          *pNode);
      bUseMatrix = false;
      collectCandidates(false);
    }
    spdlog::debug("Found {} valid transitions for {} at {}",
                  transitionProbabilities.size(),
                  *pAgent,
                  *pNode);
    if (bUseMatrix) {
      return m_extractStreet<true>(transitionProbabilities, cumulativeProbability, rng);
    }
    return m_extractStreet<false>(transitionProbabilities, cumulativeProbability, rng);
  }

  Street const* FirstOrderDynamics::m_nextStreet(const std::unique_ptr<Agent>& pAgent,
                                                 RoadJunction const* pNode,
                                                 Street const* pCurrentStreet,
                                                 std::mt19937_64& rng) {
    if (pAgent->isRandom()) {
      return m_nextRandomStreet(pAgent, pNode, pCurrentStreet, rng);
    }
    spdlog::trace("Computing m_nextStreet for {}", *pAgent);
    auto const& outgoingEdges = pNode->outgoingEdges();

    // Get current street information
    Node const* pPreviousNode{nullptr};
    static std::set<Id> const noForbiddenTurns;
    auto const* pForbiddenTurns{&noForbiddenTurns};
    // The street the agent's route takes next (set below, or no transition is returned)
    Id plannedStreetId{0};

    auto const* pStreetCurrent{m_currentStreet(pAgent, pCurrentStreet)};
    if (pStreetCurrent == nullptr) {
      // Departure from a node: the route also chooses the first street
      auto const route{m_planRouteFromNode(*pAgent, pNode->id())};
      if (route.empty()) {
        spdlog::debug(
            "No route for {} from {}. Returning no transition.", *pAgent, *pNode);
        return nullptr;
      }
      plannedStreetId = route.front();
      m_setPath(*pAgent, std::span{route}.subspan(1));
    } else {
      pPreviousNode = pStreetCurrent->source();
      pForbiddenTurns = &pStreetCurrent->forbiddenTurns();
      auto optPlannedStreetId{pAgent->nextPlannedStreetId()};
      if (!optPlannedStreetId.has_value() ||
          !m_router.isTurnAllowed(pStreetCurrent->id(), *optPlannedStreetId)) {
        // No plan (the destination is next, the agent left its route, or it was put
        // directly on a street), or a closed street ahead. Turning into the destination
        // is always the cheapest way to it; otherwise plan the rest of the trip from here
        auto const destinationId{*pAgent->destination()};
        if (m_router.isTurnAllowed(pStreetCurrent->id(), destinationId)) {
          pAgent->dropPath();
          optPlannedStreetId = destinationId;
        }
      }
      if (!optPlannedStreetId.has_value() ||
          !m_router.isTurnAllowed(pStreetCurrent->id(), *optPlannedStreetId)) {
        auto const route{m_planRoute(*pAgent, pStreetCurrent->id())};
        // A route of the current street only would mean the agent has arrived
        if (route.size() < 2) {
          spdlog::debug("No route for {} from {}. Returning no transition.",
                        *pAgent,
                        *pStreetCurrent);
          pAgent->dropPath();
          return nullptr;
        }
        m_setPath(*pAgent, std::span{route}.subspan(1));
        optPlannedStreetId = route[1];
      }
      plannedStreetId = *optPlannedStreetId;
    }
    std::span<Id const> const pathTargets{&plannedStreetId, 1};

    // Calculate transition probabilities for all valid outgoing edges. The buffer is reused
    // across calls (one per thread, as nodes evolve in parallel) to avoid an allocation per
    // routing decision
    thread_local std::vector<std::pair<Street const*, double>> transitionProbabilities;
    transitionProbabilities.clear();
    double cumulativeProbability = 0.0;

    for (auto const* pStreetOut : outgoingEdges) {
      auto const outEdgeId{pStreetOut->id()};
      if (pForbiddenTurns->contains(outEdgeId)) {
        spdlog::trace("Forbidden turn from street {} to street {}. Skipping.",
                      pAgent->streetId().value_or(0),
                      outEdgeId);
        continue;
      }

      auto const& streetOut{*pStreetOut};

      // Check if this is a valid path target for non-random agents
      bool bIsPathTarget = false;
      bIsPathTarget = std::ranges::find(pathTargets, streetOut.id()) != pathTargets.end();

      if (!this->m_errorProbability.has_value() && !bIsPathTarget) {
        continue;
      }

      double probability = 1.0;
      //std::exp(-0.5 * pStreetOut->maxSpeed() / (this->m_speedFunction(*pStreetOut)));

      // Apply error probability for non-random agents
      if (this->m_errorProbability.has_value()) {
        probability *=
            (bIsPathTarget
                 ? (1. - *m_errorProbability)
                 : *m_errorProbability /
                       static_cast<double>(outgoingEdges.size() - pathTargets.size()));
      }

      // Handle U-turns
      if (pPreviousNode != nullptr && streetOut.target() == pPreviousNode) {
        if (pNode->isRoundabout()) {
          probability *= m_uturnPenaltyFactor;  // Penalize U-turns in roundabouts
        } else if (!bIsPathTarget) {
          continue;  // No U-turns allowed in non-roundabout nodes
        }
      }

      transitionProbabilities.emplace_back(pStreetOut, probability);
      cumulativeProbability += probability;
    }

    auto const* pNextStreet{
        m_extractStreet<false>(transitionProbabilities, cumulativeProbability, rng)};
    if (pNextStreet != nullptr) {
      if (pNextStreet->id() != plannedStreetId) {
        // A deviation (see setErrorProbability): plan again at the next junction
        pAgent->dropPath();
      } else if (pStreetCurrent != nullptr && pAgent->nPlannedStreets() > 0) {
        // Entering the destination leaves nothing to advance
        pAgent->advancePath();
      }
    }
    return pNextStreet;
  }

  void FirstOrderDynamics::m_evolveStreet(Street* pStreet,
                                          RoadJunction* pNode,
                                          std::mt19937_64& rng) {
    assert(pStreet->target() == pNode);
    auto const nLanes = pStreet->nLanes();
    // Enqueue moving agents if their free time is up
    while (!pStreet->movingAgents().empty()) {
      auto const& pAgent{pStreet->movingAgents().top()};
      // movingAgents is a min-heap on free time: as soon as the earliest agent is still
      // travelling the street, none of the others can be done either.
      if (pAgent->freeTime() > this->time_step()) {
        break;
      }
      pAgent->setSpeed(0.);
      // The agent arrives at the end of its destination street, without queueing
      if (!pAgent->isRandom() && pAgent->destination() == pStreet->id()) {
        spdlog::debug("{} has arrived at the end of its destination street {}",
                      *pAgent,
                      pStreet->id());
        auto pArrived{
            this->m_removeAgent<true>(pStreet->dequeueArrivedAgent(this->time_step()))};
        if (m_reinsertAgents) {
          pArrived->reset(this->time_step());
          m_agentsToReinsert.push_back(std::move(pArrived));
        }
        continue;
      }
      auto const* pNextStreet{this->m_nextStreet(pAgent, pNode, pStreet, rng)};
      if (pNextStreet == nullptr) {
        spdlog::debug("No next street found for agent {} at node {}",
                      *pAgent,
                      pStreet->target()->id());
        if (pAgent->isRandom()) {
          std::uniform_int_distribution<size_t> laneDist{0,
                                                         static_cast<size_t>(nLanes - 1)};
          pStreet->enqueue(laneDist(rng));
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
      pAgent->setNextStreetId(pNextStreet->id());
      if (nLanes == 1) {
        pStreet->enqueue(0);
        continue;
      }
      auto const direction{pNextStreet->turnDirection(pStreet->angle())};
      std::vector<size_t> validLanes;
      auto const& lanes = pStreet->lanes();
      for (size_t laneIndex = 0; laneIndex < lanes.size(); ++laneIndex) {
        auto const laneDirection{lanes[laneIndex].direction};
        switch (direction) {
          case Direction::ANY:
            validLanes.push_back(laneIndex);
            break;
          case Direction::UTURN:
          case Direction::LEFT:
            if (laneDirection == Direction::LEFT ||
                laneDirection == Direction::LEFTANDSTRAIGHT) {
              validLanes.push_back(laneIndex);
            }
            break;
          case Direction::STRAIGHT:
            if (laneDirection == Direction::STRAIGHT ||
                laneDirection == Direction::LEFTANDSTRAIGHT ||
                laneDirection == Direction::RIGHTANDSTRAIGHT) {
              validLanes.push_back(laneIndex);
            }
            break;
          case Direction::RIGHT:
            if (laneDirection == Direction::RIGHT ||
                laneDirection == Direction::RIGHTANDSTRAIGHT) {
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
      if (validLanes.empty()) {
        // No lane of this street serves the required direction: fall back to a
        // uniform choice rather than normalising an all-zero weight vector.
        spdlog::debug(
            "No lane on street {} maps direction {}; falling back to a uniform lane "
            "choice.",
            pStreet->id(),
            directionToString.at(static_cast<std::size_t>(direction)));
        std::uniform_int_distribution<size_t> fallbackDist{
            0, static_cast<size_t>(nLanes - 1)};
        pStreet->enqueue(fallbackDist(rng));
        continue;
      }
      // If all weights are the same, make the last 0
      if (std::ranges::all_of(weights, [&](double w) {
            return std::abs(w - weights.front()) < std::numeric_limits<double>::epsilon();
          })) {
        weights.back() = 0.;
        if (nLanes > 2) {
          weights.front() = 0.;
        }
      }
      // Normalize the weights
      auto const sum = std::accumulate(weights.begin(), weights.end(), 0.);
      if (!(sum > 0.)) {
        std::uniform_int_distribution<size_t> fallbackDist{
            0, static_cast<size_t>(nLanes - 1)};
        pStreet->enqueue(fallbackDist(rng));
        continue;
      }
      for (auto& w : weights) {
        w /= sum;
      }
      std::discrete_distribution<size_t> laneDist{weights.begin(), weights.end()};
      pStreet->enqueue(laneDist(rng));
    }
    std::uniform_real_distribution<double> uniformDist{0., 1.};
    for (auto queueIndex = 0; queueIndex < nLanes; ++queueIndex) {
      if (pStreet->queue(queueIndex).empty()) {
        continue;
      }
      pStreet->refillFlowAccumulator(queueIndex, this->time_step());
      if (!pStreet->lane(queueIndex).flowAccumulator.canPass()) {
        spdlog::trace("Skipping lane {} of {} due to transport capacity {}",
                      queueIndex,
                      *pStreet,
                      pStreet->transportCapacity());
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
        auto const timeTolerance{*m_timeToleranceFactor *
                                 std::ceil(pStreet->length() / pStreet->maxSpeed())};
        if (timeDiff > timeTolerance) {
          spdlog::debug(
              "Time-step {} - {} currently on {} ({} turn - Traffic Light? {}), "
              "has been still for more than {} seconds ({} seconds). Killing it.",
              this->time_step(),
              *pAgentTemp,
              *pStreet,
              directionToString.at(pStreet->lane(queueIndex).direction),
              pNode->isTrafficLight(),
              timeTolerance,
              timeDiff);
          // Kill the agent
          this->m_removeAgent<false>(pStreet->dequeue(queueIndex, this->time_step()));
          continue;
        }
      }
      pAgentTemp->setSpeed(0.);
      auto* destinationNode{pNode};
      if (destinationNode->isFull()) {
        spdlog::trace("Skipping due to full destination node {}", *destinationNode);
        continue;
      }
      if (destinationNode->isTrafficLight()) {
        auto& tl = dynamic_cast<TrafficLight&>(*destinationNode);
        auto const direction{pStreet->lane(queueIndex).direction};
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
          spdlog::debug("Checking priorities for {}", *pStreet);
          auto const& thisDirection{this->graph()
                                        .edge(*(pAgentTemp->nextStreetId()))
                                        .turnDirection(pStreet->angle())};
          if (!intersection.streetPriorities().contains(pStreet->id())) {
            // I have to check if the agent has right of way
            for (auto const* pStreetTemp : destinationNode->ingoingEdges()) {
              if (pStreetTemp == pStreet) {
                continue;
              }
              if (pStreetTemp->nExitingAgents() == 0) {
                continue;
              }
              if (intersection.streetPriorities().contains(pStreetTemp->id())) {
                spdlog::debug("Skipping agent emission from {} due to right of way.",
                              *pStreet);
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
                  auto const streetIdOpt2{pAgentTemp2->streetId()};
                  auto const nextStreetIdOpt2{pAgentTemp2->nextStreetId()};
                  if (!nextStreetIdOpt2.has_value() || !streetIdOpt2.has_value()) {
                    continue;
                  }
                  // Queued agents are on the street they are queued on
                  assert(*streetIdOpt2 == pStreetTemp->id());
                  auto const& otherDirection{this->graph()
                                                 .edge(*nextStreetIdOpt2)
                                                 .turnDirection(pStreetTemp->angle())};
                  if (otherDirection < Direction::LEFT) {
                    spdlog::debug(
                        "Skipping agent emission from {} due to right of "
                        "way with other agents.",
                        *pStreet);
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
                auto const streetIdOpt2{pAgentTemp2->streetId()};
                auto const nextStreetIdOpt2{pAgentTemp2->nextStreetId()};
                if (!streetIdOpt2.has_value() || !nextStreetIdOpt2.has_value()) {
                  continue;
                }
                // Queued agents are on the street they are queued on
                assert(*streetIdOpt2 == pStreetTemp->id());
                auto const& otherDirection{this->graph()
                                               .edge(*nextStreetIdOpt2)
                                               .turnDirection(pStreetTemp->angle())};
                if (otherDirection < thisDirection) {
                  spdlog::debug(
                      "Skipping agent emission from {} due to right of "
                      "way with other agents.",
                      *pStreet);
                  bCanPass = false;
                  break;
                }
              }
            }
          }
        }
        if (!bCanPass) {
          spdlog::debug("Skipping agent emission from {} due to right of way", *pStreet);
          continue;
        }
      }
      bool bArrived{false};
      if (!(uniformDist(rng) <
            m_passageProbability.value_or(std::numeric_limits<double>::max()))) {
        if (pAgentTemp->isRandom()) {
          bArrived = true;
        } else {
          spdlog::debug(
              "Skipping agent emission from {} due to passage "
              "probability",
              *pStreet);
          continue;
        }
      }
      if (pAgentTemp->isRandom()) {
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
          m_agentsToReinsert.push_back(std::move(pAgent));
        }
        continue;
      }
      if (!pAgentTemp->streetId().has_value()) {
        throw std::runtime_error(
            std::format("Agent {} has no current street ID at node {}. Cannot proceed.",
                        pAgentTemp->id(),
                        destinationNode->id()));
      }
      auto* nextStreet{&this->graph().edge(*(pAgentTemp->nextStreetId()))};
      if (nextStreet->isFull()) {
        spdlog::debug(
            "Skipping agent emission from {} due to full "
            "next street: {}",
            *pStreet,
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
      assert(nextStreet->source() == destinationNode);
      if (destinationNode->isIntersection()) {
        auto& intersection = dynamic_cast<Intersection&>(*destinationNode);
        auto const delta{nextStreet->deltaAngle(pStreet->angle())};
        intersection.addAgent(delta, std::move(pAgent));
      } else if (destinationNode->isRoundabout()) {
        auto& roundabout = dynamic_cast<Roundabout&>(*destinationNode);
        roundabout.enqueue(std::move(pAgent));
      } else {
        spdlog::warn(
            "{} cannot host agents (it is neither an intersection nor a roundabout). "
            "Killing the agent coming from street {}.",
            *destinationNode,
            pStreet->id());
        this->m_removeAgent<false>(std::move(pAgent));
      }
    }
  }

  void FirstOrderDynamics::m_evolveNode(RoadJunction* pNode) {
    bool bIdle{true};
    if (pNode->isIntersection()) {
      bIdle = static_cast<Intersection const&>(*pNode).nAgents() == 0;
    } else if (pNode->isRoundabout()) {
      bIdle = static_cast<Roundabout const&>(*pNode).agents().empty();
    }
    if (bIdle) {
      // The gate is left untouched: its next refill catches up on the idle time steps
      return;
    }
    auto& flowAccumulator{pNode->flowAccumulator()};
    flowAccumulator.refill(pNode->transportCapacity(), this->time_step());
    while (flowAccumulator.canPass()) {
      bool bReleased{false};
      if (pNode->isIntersection()) {
        auto& intersection = dynamic_cast<Intersection&>(*pNode);
        if (intersection.agents().empty()) {
          return;
        }
        for (auto it{intersection.agents().begin()}; it != intersection.agents().end();) {
          auto& pAgent{it->second};
          if (!pAgent->nextStreetId().has_value()) {
            throw std::runtime_error(std::format(
                "{} has no next street ID at {}. Cannot proceed.", *pAgent, *pNode));
          }
          auto* nextStreet{&this->graph().edge(*(pAgent->nextStreetId()))};
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
            auto const count{++m_turnCounts[*(pAgent->streetId())][nextStreet->id()]};
            spdlog::trace("Incremented turn count for {} -> {}: {}",
                          *(pAgent->streetId()),
                          nextStreet->id(),
                          count);
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
          bReleased = true;
          break;
        }
      } else if (pNode->isRoundabout()) {
        auto& roundabout = dynamic_cast<Roundabout&>(*pNode);
        if (roundabout.agents().empty()) {
          return;
        }
        auto const& pAgentTemp{roundabout.agents().front()};
        if (!pAgentTemp->nextStreetId().has_value()) {
          throw std::runtime_error(std::format(
              "Agent {} has no next street ID at roundabout {}. Cannot proceed.",
              *pAgentTemp,
              *pNode));
        }
        auto* nextStreet{&this->graph().edge(*(pAgentTemp->nextStreetId()))};
        if (!(nextStreet->isFull())) {
          if (!m_turnCounts.empty() && pAgentTemp->streetId().has_value()) {
            auto const count{++m_turnCounts[*(pAgentTemp->streetId())][nextStreet->id()]};
            spdlog::trace("Incremented turn count for {} -> {}: {}",
                          *(pAgentTemp->streetId()),
                          nextStreet->id(),
                          count);
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
          bReleased = true;
        }
      }
      if (!bReleased) {
        // Every agent is blocked: trying again in this time step would not change that
        return;
      }
      flowAccumulator.consume();
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
    // Whether a node can host one more agent, given the departures already decided from it
    // in this time step: the agents only enter their source node at the end
    std::unordered_map<RoadJunction const*, std::size_t> nDepartures;
    auto const canHost = [&nDepartures](RoadJunction const* pNode) {
      auto const nDecided{nDepartures[pNode]};
      if (pNode->isIntersection()) {
        return static_cast<Intersection const&>(*pNode).nAgents() + nDecided <
               pNode->capacity();
      }
      if (pNode->isRoundabout()) {
        return static_cast<Roundabout const&>(*pNode).agents().size() + nDecided <
               pNode->capacity();
      }
      return !pNode->isFull();
    };
    // 1. Select, in order, the agents which depart in this time step
    std::vector<std::size_t> departingIndices;
    // Indices in m_agents of the departing agents to route, and whether a route was found
    std::vector<std::size_t> agentsToRoute;
    std::vector<std::uint8_t> bRoutable(m_agents.size(), 1);
    for (std::size_t i{0}; i < m_agents.size(); ++i) {
      auto& pAgent{m_agents[i]};
      if (!pAgent->srcNodeId().has_value()) {
        auto const nodeIndex{nodeDist(this->m_generator)};
        if (m_nodeIndices.size() == this->graph().nNodes()) {
          // Nodes are never removed, so m_nodeIndices still follows the nodes'
          // iteration order: no need to walk the map
          pAgent->setSrcNodeId(m_nodeIndices[nodeIndex]);
        } else {
          auto nodeIt{this->graph().nodes().begin()};
          std::advance(nodeIt, nodeIndex);
          pAgent->setSrcNodeId((*nodeIt).first);
        }
      }
      auto const* pSourceNode{&this->graph().node(*(pAgent->srcNodeId()))};
      if (!canHost(pSourceNode)) {
        spdlog::debug("Skipping {} due to full source {}", *pAgent, *pSourceNode);
        continue;
      }
      Street const* nextStreet{nullptr};
      if (!pAgent->nextStreetId().has_value()) {
        spdlog::debug("No next street id, generating a new one");
        nextStreet = this->m_nextStreet(pAgent, pSourceNode, nullptr, this->m_generator);
        if (nextStreet == nullptr) {
          spdlog::debug(
              "No next street found for agent {} at node {}", *pAgent, pSourceNode->id());
          if (pAgent->isRandom()) {
            this->m_removeAgent<false>(std::move(pAgent));
          } else {
            m_dropPendingAgent(pAgent);
          }
          continue;
        }
        pAgent->setNextStreetId(nextStreet->id());
      } else {
        nextStreet = &this->graph().edge(*(pAgent->nextStreetId()));
      }
      if (!pAgent->srcStreetId().has_value()) {
        // Remember where the trip starts, to restart it from there on reinsertion
        pAgent->setSrcStreetId(*pAgent->nextStreetId());
      }
      spdlog::debug("Checking next street for {}", *pAgent);
      if (nextStreet->isFull()) {
        spdlog::debug("Skipping {} due to full input {}", *pAgent, *nextStreet);
        if (pAgent->isIntelligent()) {
          // Plan again on the traffic of the actual departure
          pAgent->dropPath();
        }
        continue;
      }
      ++nDepartures[pSourceNode];
      departingIndices.push_back(i);
      if (!pAgent->isRandom() && pAgent->nPlannedStreets() == 0) {
        agentsToRoute.push_back(i);
      }
    }
    // 2. Plan the routes of the departing agents: each search is independent
    if (!agentsToRoute.empty()) {
      this->m_taskArena.execute([&] {
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, agentsToRoute.size()),
                          [&](tbb::blocked_range<std::size_t> const& range) {
                            for (auto i{range.begin()}; i != range.end(); ++i) {
                              auto const index{agentsToRoute[i]};
                              auto& agent{*m_agents[index]};
                              auto const route{m_planRoute(agent, *agent.nextStreetId())};
                              bRoutable[index] = route.empty() ? 0 : 1;
                              if (!route.empty()) {
                                m_setPath(agent, std::span{route}.subspan(1));
                              }
                            }
                          });
      });
    }
    // 3. Let the departing agents enter their source node, in order. Agents leaving the
    // list are moved out, leaving a null slot: the list is compacted once at the end instead
    // of erasing (and shifting the tail) for each of them
    for (auto const i : departingIndices) {
      auto& pAgent{m_agents[i]};
      if (bRoutable[i] == 0) {
        m_dropPendingAgent(pAgent);
        continue;
      }
      auto* pSourceNode{&this->graph().node(*(pAgent->srcNodeId()))};
      // The agent enters the simulation only now: re-stamp its spawn time so that the
      // time it spent queued in m_agents, waiting for a free slot, is not counted as
      // travel time.
      pAgent->setSpawnTime(this->time_step());
      if (pSourceNode->isIntersection()) {
        auto& intersection = dynamic_cast<Intersection&>(*pSourceNode);
        intersection.addAgent(0., std::move(pAgent));
      } else if (pSourceNode->isRoundabout()) {
        auto& roundabout = dynamic_cast<Roundabout&>(*pSourceNode);
        roundabout.enqueue(std::move(pAgent));
      } else {
        spdlog::warn(
            "{} cannot host agents (it is neither an intersection nor a roundabout). "
            "Killing agent {}.",
            *pSourceNode,
            pAgent->id());
        this->m_removeAgent<false>(std::move(pAgent));
      }
    }
    std::erase(m_agents, nullptr);
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
  void FirstOrderDynamics::setIntelligentAgentsFraction(double const intelligentFraction) {
    if (intelligentFraction < 0. || intelligentFraction > 1.) {
      throw std::invalid_argument(std::format(
          "The intelligent fraction ({}) must be in [0, 1]", intelligentFraction));
    }
    m_intelligentFraction = intelligentFraction;
  }
  void FirstOrderDynamics::setEdgeWeight(std::string_view const weight,
                                         double const distanceCost,
                                         std::optional<double> const sigma) {
    m_router.setEdgeWeight(weight, distanceCost, sigma);
    if (m_router.isBuilt()) {
      // Validate the new cost function, e.g. a missing custom attribute, right away
      this->m_taskArena.execute([this] { m_router.updateCurrentWeights(); });
    }
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
  std::vector<Id> FirstOrderDynamics::m_streetIds(Id const id,
                                                  bool const bDestination) const {
    if (m_mode == DynamicsMode::EDGES) {
      return {id};
    }
    auto const& node{this->graph().node(id)};
    // Agents start from an origin node, and arrive at a destination node at the end of the
    // street they travel last
    auto const& edges{bDestination ? node.ingoingEdges() : node.outgoingEdges()};
    std::vector<Id> streetIds;
    streetIds.reserve(edges.size());
    std::ranges::transform(edges, std::back_inserter(streetIds), [](auto const* pStreet) {
      return pStreet->id();
    });
    if (streetIds.empty()) {
      spdlog::warn("Node {} has no {} streets. Skipping it.",
                   id,
                   bDestination ? "ingoing" : "outgoing");
    }
    return streetIds;
  }
  std::unordered_map<Id, double> FirstOrderDynamics::m_toStreetWeights(
      std::unordered_map<Id, double> const& weights, bool const bDestination) const {
    if (m_mode == DynamicsMode::EDGES) {
      return weights;
    }
    std::unordered_map<Id, double> streetWeights;
    streetWeights.reserve(weights.size());
    for (auto const& [nodeId, weight] : weights) {
      auto const streetIds{m_streetIds(nodeId, bDestination)};
      for (auto const streetId : streetIds) {
        streetWeights[streetId] += weight / static_cast<double>(streetIds.size());
      }
    }
    if (streetWeights.empty()) {
      throw std::invalid_argument(
          std::format("None of the given nodes has {} streets to use as {}.",
                      bDestination ? "ingoing" : "outgoing",
                      bDestination ? "destination" : "origin"));
    }
    return streetWeights;
  }
  void FirstOrderDynamics::setMode(DynamicsMode const mode) {
    if (mode == m_mode) {
      return;
    }
    if (!m_origins.empty() || !m_destinations.empty() || !m_ODs.empty() ||
        !m_originToDestinations.empty()) {
      throw std::runtime_error(
          "FirstOrderDynamics::setMode: the mode must be set before any origin, "
          "destination or OD pair.");
    }
    m_mode = mode;
  }
  void FirstOrderDynamics::setOrigins(std::unordered_map<Id, double> const& origins) {
    m_origins.clear();
    if (origins.empty()) {
      // If no origins are provided, use all streets with uniform weights
      if (this->graph().nEdges() == 0) {
        throw std::runtime_error(
            "FirstOrderDynamics::setOrigins: no origins given and the graph has no "
            "edges to fall back on.");
      }
      m_origins.reserve(this->graph().nEdges());
      auto const UNIFORM_WEIGHT{1. / this->graph().nEdges()};
      for (auto const& edgePair : this->graph().edges()) {
        m_origins.push_back({edgePair.first, UNIFORM_WEIGHT});
      }
      return;
    }
    // In EDGES mode the given map is used as is, so that its iteration order (which the
    // seeded random choices depend on) is kept: copying an unordered_map may change it
    std::unordered_map<Id, double> nodeStreetOrigins;
    auto const& streetOrigins{
        m_mode == DynamicsMode::NODES
            ? (nodeStreetOrigins = m_toStreetWeights(origins, false))
            : origins};
    m_origins.reserve(streetOrigins.size());
    auto const sumWeights = std::accumulate(
        streetOrigins.begin(), streetOrigins.end(), 0., [](double sum, auto const& pair) {
          return sum + pair.second;
        });
    if (sumWeights <= 0.) {
      throw std::invalid_argument(
          std::format("The sum of the weights ({}) must be positive", sumWeights));
    }
    // Dividing by a sum of exactly 1 leaves the weights unchanged
    std::ranges::transform(
        streetOrigins, std::back_inserter(m_origins), [sumWeights](auto const& pair) {
          return WeightedId{.id = pair.first, .weight = pair.second / sumWeights};
        });
  }
  void FirstOrderDynamics::setDestinations(
      std::unordered_map<Id, double> const& destinations) {
    // In EDGES mode the given map is used as is, to keep its iteration order (see setOrigins)
    std::unordered_map<Id, double> nodeStreetDestinations;
    auto const& streetDestinations{
        m_mode == DynamicsMode::NODES
            ? (nodeStreetDestinations = m_toStreetWeights(destinations, true))
            : destinations};
    m_destinationStreets.clear();
    m_bDestinationsChecked = false;
    m_destinations.clear();
    m_destinations.reserve(streetDestinations.size());
    double sumWeights{0.};
    for (auto const& [destinationId, weight] : streetDestinations) {
      sumWeights += weight;
      m_destinationStreets.insert(destinationId);
    }
    if (sumWeights <= 0.) {
      throw std::invalid_argument(
          std::format("The sum of the weights ({}) must be positive", sumWeights));
    }
    // Dividing by a sum of exactly 1 leaves the weights unchanged
    std::ranges::transform(streetDestinations,
                           std::back_inserter(m_destinations),
                           [sumWeights](auto const& pair) {
                             return WeightedId{.id = pair.first,
                                               .weight = pair.second / sumWeights};
                           });
  }
  void FirstOrderDynamics::setTransitionMatrix(
      std::unordered_map<Id, std::unordered_map<Id, double>> const& transitionMatrix) {
    constexpr double TOLERANCE{1e-9};
    std::unordered_map<Id, std::unordered_map<Id, double>> validatedMatrix;
    validatedMatrix.reserve(transitionMatrix.size());
    for (auto const& [srcStreetId, row] : transitionMatrix) {
      if (!this->graph().edges().contains(srcStreetId)) {
        spdlog::warn(
            "Transition matrix: street {} does not exist in the road network. Ignoring "
            "its row.",
            srcStreetId);
        continue;
      }
      auto const& srcStreet{this->graph().edge(srcStreetId)};
      auto const outgoingEdges{srcStreet.target()->outgoingEdges()};
      std::unordered_map<Id, double> validatedRow;
      validatedRow.reserve(row.size());
      double sumWeights{0.};
      double redistributableWeight{0.};
      for (auto const& [dstStreetId, weight] : row) {
        if (weight < 0.) {
          throw std::invalid_argument(std::format(
              "The transition probability from street {} to street {} ({}) must be "
              "non-negative",
              srcStreetId,
              dstStreetId,
              weight));
        }
        if (!this->graph().edges().contains(dstStreetId)) {
          spdlog::warn(
              "Transition matrix: street {} (target of {}) does not exist in the road "
              "network. Ignoring this transition.",
              dstStreetId,
              srcStreetId);
          continue;
        }
        if (std::ranges::find(outgoingEdges, dstStreetId, &Street::id) ==
            outgoingEdges.end()) {
          spdlog::warn(
              "Transition matrix: street {} is not an outgoing edge of node {}, i.e. the "
              "target of street {}. Ignoring this transition.",
              dstStreetId,
              srcStreet.target()->id(),
              srcStreetId);
          continue;
        }
        if (srcStreet.forbiddenTurns().contains(dstStreetId)) {
          spdlog::debug(
              "Transition matrix: the turn from street {} to street {} is forbidden, "
              "which random agents cannot take. Its probability will be redistributed "
              "among the other transitions of street {}.",
              srcStreetId,
              dstStreetId,
              srcStreetId);
          redistributableWeight += weight;
          continue;
        }
        if (this->graph().edge(dstStreetId).target() == srcStreet.source()) {
          spdlog::debug(
              "Transition matrix: the transition from street {} to street {} is a "
              "U-turn, which random agents cannot take. Its probability will be "
              "redistributed among the other transitions of street {}.",
              srcStreetId,
              dstStreetId,
              srcStreetId);
          redistributableWeight += weight;
          continue;
        }
        sumWeights += weight;
        validatedRow.emplace(dstStreetId, weight);
      }
      if (redistributableWeight > 0.) {
        if (validatedRow.empty()) {
          spdlog::warn(
              "Transition matrix: street {} has no other usable transition to "
              "redistribute the forbidden-turn/U-turn probability onto. It will be "
              "dropped.",
              srcStreetId);
        } else if (sumWeights > 0.) {
          double const scale{(sumWeights + redistributableWeight) / sumWeights};
          for (auto& [dstStreetId, weight] : validatedRow) {
            weight *= scale;
          }
          sumWeights += redistributableWeight;
        } else {
          double const share{redistributableWeight /
                             static_cast<double>(validatedRow.size())};
          for (auto& [dstStreetId, weight] : validatedRow) {
            weight = share;
          }
          sumWeights = redistributableWeight;
        }
      }
      if (sumWeights > 1. + TOLERANCE) {
        throw std::invalid_argument(std::format(
            "The transition probabilities of street {} sum up to {}, which is greater "
            "than 1",
            srcStreetId,
            sumWeights));
      }
      if (validatedRow.empty()) {
        spdlog::warn(
            "Transition matrix: street {} has no usable transition left. Random agents "
            "will use the uniform behaviour there.",
            srcStreetId);
        continue;
      }
      validatedMatrix.emplace(srcStreetId, std::move(validatedRow));
    }
    if (!validatedMatrix.empty() && validatedMatrix.size() < this->graph().nEdges()) {
      spdlog::warn(
          "Transition matrix: {} out of {} streets have no transition probabilities. "
          "Random agents will use the uniform behaviour there.",
          this->graph().nEdges() - validatedMatrix.size(),
          this->graph().nEdges());
    }
    m_transitionMatrix = std::move(validatedMatrix);
  }
  void FirstOrderDynamics::importTransitionMatrixFromJSON(
      std::string_view const fileName) {
    simdjson::dom::parser parser;
    simdjson::dom::element root;
    auto const error = parser.load(fileName).get(root);
    if (error) {
      throw std::runtime_error(
          std::format("Failed to load the transition matrix file '{}': {}",
                      fileName,
                      simdjson::error_message(error)));
    }
    simdjson::dom::object rootObject;
    if (root.get_object().get(rootObject)) {
      throw std::runtime_error(std::format(
          "Invalid transition matrix file '{}': root element is not an object",
          fileName));
    }
    auto const parseId = [&fileName](std::string_view const key) -> Id {
      Id id{0};
      auto const [ptr, ec] = std::from_chars(key.data(), key.data() + key.size(), id);
      if (ec != std::errc{} || ptr != key.data() + key.size()) {
        throw std::invalid_argument(std::format(
            "Invalid street id '{}' in the transition matrix file '{}'", key, fileName));
      }
      return id;
    };
    std::unordered_map<Id, std::unordered_map<Id, double>> transitionMatrix;
    for (auto const& [srcKey, rowElement] : rootObject) {
      simdjson::dom::object rowObject;
      if (rowElement.get_object().get(rowObject)) {
        throw std::runtime_error(std::format(
            "Invalid transition matrix file '{}': the value of '{}' is not an "
            "object",
            fileName,
            srcKey));
      }
      auto const srcStreetId{parseId(srcKey)};
      std::unordered_map<Id, double> row;
      for (auto const& [dstKey, weightElement] : rowObject) {
        double weight;
        if (weightElement.get_double().get(weight)) {
          throw std::runtime_error(std::format(
              "Invalid transition matrix file '{}': the probability of '{}' -> '{}' is "
              "not a number",
              fileName,
              srcKey,
              dstKey));
        }
        row.emplace(parseId(dstKey), weight);
      }
      transitionMatrix.emplace(srcStreetId, std::move(row));
    }
    this->setTransitionMatrix(transitionMatrix);
    spdlog::info("Finished importing the transition matrix from '{}' ({} streets).",
                 fileName,
                 m_transitionMatrix.size());
  }
  void FirstOrderDynamics::initTurnCounts() {
    if (!m_turnCounts.empty()) {
      throw std::runtime_error(
          std::format("Turn counts have already been initialized (current size: {}).",
                      m_turnCounts.size()));
    }
    for (auto const& [edgeId, pEdge] : this->graph().edges()) {
      for (auto const* pNextEdge : pEdge->target()->outgoingEdges()) {
        spdlog::debug(
            "Initializing turn count for edge {} -> {}", edgeId, pNextEdge->id());
        m_turnCounts[edgeId][pNextEdge->id()] = 0;
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
      for (auto const* pNextEdge : pEdge->target()->outgoingEdges()) {
        m_turnCounts[edgeId][pNextEdge->id()] = 0;
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

  void FirstOrderDynamics::setDestinations(std::initializer_list<Id> destinations) {
    this->setDestinations(std::vector<Id>(destinations));
  }
  void FirstOrderDynamics::setODs(std::vector<ODEntry> const& ODs) {
    // Translate the ids into street ids, splitting each weight over the expanded pairs
    std::vector<ODEntry> streetODs;
    streetODs.reserve(ODs.size());
    for (auto const& [originId, destinationId, weight] : ODs) {
      auto const originStreetIds{m_streetIds(originId, false)};
      auto const destinationStreetIds{m_streetIds(destinationId, true)};
      if (originStreetIds.empty() || destinationStreetIds.empty()) {
        continue;
      }
      auto const pairWeight{weight / static_cast<double>(originStreetIds.size() *
                                                         destinationStreetIds.size())};
      for (auto const originStreetId : originStreetIds) {
        for (auto const destinationStreetId : destinationStreetIds) {
          streetODs.push_back({originStreetId, destinationStreetId, pairWeight});
        }
      }
    }
    if (!ODs.empty() && streetODs.empty()) {
      throw std::invalid_argument(
          "None of the given OD pairs has an origin node with outgoing streets and a "
          "destination node with ingoing streets.");
    }
    m_ODs.clear();
    m_ODCumulativeWeights.clear();
    m_bDestinationsChecked = false;
    m_ODs.reserve(streetODs.size());
    auto const sumWeights = std::accumulate(
        streetODs.begin(), streetODs.end(), 0., [this](double sum, ODEntry const& od) {
          // Add the destinations while summing weights
          m_destinationStreets.insert(od.destination);
          return sum + od.weight;
        });
    if (sumWeights <= 0.) {
      throw std::invalid_argument(
          std::format("The sum of the weights ({}) must be positive", sumWeights));
    }
    if (sumWeights == 1.) {
      std::ranges::copy(streetODs, std::back_inserter(m_ODs));
    } else {
      // Copy but divide by weights sum
      std::ranges::transform(
          streetODs, std::back_inserter(m_ODs), [sumWeights](ODEntry const& od) {
            return ODEntry{.origin = od.origin,
                           .destination = od.destination,
                           .weight = od.weight / sumWeights};
          });
    }
  }
  void FirstOrderDynamics::setConditionalODs(
      std::unordered_map<Id, ConditionalOrigin> const& conditionalODs) {
    m_originToDestinations.clear();
    m_originToDestinations.reserve(conditionalODs.size());
    m_bDestinationsChecked = false;
    std::unordered_map<Id, double> originWeights;
    for (auto const& [originId, conditionalOrigin] : conditionalODs) {
      auto const& [originWeight, originDestinations] = conditionalOrigin;
      // Translate the destination ids into street ids, splitting each weight
      std::vector<WeightedId> streetDestinations;
      streetDestinations.reserve(originDestinations.size());
      for (auto const& [destinationId, weight] : originDestinations) {
        auto const destinationStreetIds{m_streetIds(destinationId, true)};
        for (auto const destinationStreetId : destinationStreetIds) {
          streetDestinations.push_back(
              {destinationStreetId,
               weight / static_cast<double>(destinationStreetIds.size())});
        }
      }
      auto sumDestinationWeights = std::accumulate(
          streetDestinations.begin(),
          streetDestinations.end(),
          0.,
          [](double sum, WeightedId const& dest) { return sum + dest.weight; });
      if (sumDestinationWeights <= 0.) {
        throw std::invalid_argument(std::format(
            "The sum of the destination weights ({}) for origin {} must be positive",
            sumDestinationWeights,
            originId));
      }
      for (auto& [destinationId, weight] : streetDestinations) {
        m_destinationStreets.insert(destinationId);
        weight /= sumDestinationWeights;
      }
      // Each origin street gets the origin's destinations (setOrigins splits its weight)
      auto const originStreetIds{m_streetIds(originId, false)};
      if (originStreetIds.empty()) {
        continue;
      }
      originWeights[originId] = originWeight;
      for (auto const originStreetId : originStreetIds) {
        m_originToDestinations[originStreetId] = streetDestinations;
      }
    }
    if (!conditionalODs.empty() && originWeights.empty()) {
      throw std::invalid_argument(
          "None of the given origin nodes has outgoing streets to use as origin.");
    }
    this->setOrigins(std::move(originWeights));
  }

  void FirstOrderDynamics::m_rebuildRouter() {
    spdlog::debug("Rebuilding the routing graph");
    this->m_taskArena.execute([this] { m_router.build(this->graph()); });
    m_routerRevision = Road::routingRevision();
    m_routerNodes = this->graph().nNodes();
    // The waiting agents plan their routes on the new routing graph
    for (auto& pAgent : m_agents) {
      pAgent->dropPath();
    }
    m_checkDestinations();
  }
  void FirstOrderDynamics::m_checkDestinations() {
    m_bDestinationsChecked = true;
    // Agents drive their destination street: it must exist, be open, and be entered from an
    // open street by an allowed turn
    std::vector<Id> unreachable;
    for (auto const destinationId : m_destinationStreets) {
      if (!this->graph().edges().contains(destinationId)) {
        spdlog::error("The destination street {} does not exist", destinationId);
        unreachable.push_back(destinationId);
        continue;
      }
      auto const& ingoingEdges{
          this->graph().edge(destinationId).source()->ingoingEdges()};
      if (std::ranges::any_of(ingoingEdges, [&](Street const* pStreet) {
            return pStreet->isActive() &&
                   m_router.isTurnAllowed(pStreet->id(), destinationId);
          })) {
        continue;
      }
      spdlog::error("No path found to destination street {}", destinationId);
      unreachable.push_back(destinationId);
    }
    if (unreachable.empty()) {
      return;
    }
    auto const strIds = std::accumulate(unreachable.begin(),
                                        unreachable.end(),
                                        std::string{},
                                        [](std::string const& a, Id const& b) {
                                          return a.empty() ? std::to_string(b)
                                                           : a + ", " + std::to_string(b);
                                        });
    spdlog::error("Removing {} destinations with no valid path from the dynamics ({}).",
                  unreachable.size(),
                  strIds);
    for (auto const destination : unreachable) {
      std::erase_if(m_ODs, [destination](ODEntry const& od) {
        return od.destination == destination;
      });
      std::erase_if(m_destinations, [destination](WeightedId const& dest) {
        return dest.id == destination;
      });
      std::erase_if(m_origins, [destination](WeightedId const& origin) {
        return origin.id == destination;
      });
      for (auto& [origin, destinations] : m_originToDestinations) {
        std::erase_if(destinations, [destination](WeightedId const& dest) {
          return dest.id == destination;
        });
      }
      // Remove all origins in m_originToDestinations that have no destinations left
      std::erase_if(m_originToDestinations, [this](auto const& pair) {
        bool const bErase{pair.second.empty()};
        if (bErase) {
          std::erase_if(m_origins, [&pair](WeightedId const& origin) {
            return origin.id == pair.first;
          });
        }
        return bErase;
      });
      m_destinationStreets.erase(destination);
    }
    m_ODCumulativeWeights.clear();
  }

  void FirstOrderDynamics::addAgentsUniformly(std::size_t nAgents,
                                              std::optional<Id> optDestinationId) {
    m_nAddedAgents += nAgents;
    if (m_timeToleranceFactor.has_value() && !m_agents.empty()) {
      auto const nStagnantAgents{m_agents.size()};
      spdlog::debug(
          "Removing {} stagnant agents that were not inserted since the previous call to "
          "addAgentsUniformly().",
          nStagnantAgents);
      m_agents.clear();
      m_nAgents -= nStagnantAgents;
      m_nKilledAgents += nStagnantAgents;
    }
    if (optDestinationId.has_value() &&
        !this->graph().edges().contains(*optDestinationId)) {
      throw std::invalid_argument(
          std::format("Cannot add agents with destination {}: the street does not exist",
                      *optDestinationId));
    }
    bool const bRandomDestination{!optDestinationId.has_value() &&
                                  !m_destinationStreets.empty()};
    auto destinationId{optDestinationId};
    std::uniform_int_distribution<std::size_t> destinationDist{
        0, m_destinationStreets.size() - 1};
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
      if (bRandomDestination) {
        auto destinationIt{m_destinationStreets.cbegin()};
        std::advance(destinationIt, destinationDist(this->m_generator));
        destinationId = *destinationIt;
      }
      auto streetIt = this->graph().edges().begin();
      while (true) {
        auto step = streetDist(this->m_generator);
        std::advance(streetIt, step);
        if (!((*streetIt).second->isFull())) {
          break;
        }
        streetIt = this->graph().edges().begin();
      }
      auto const [streetId, street] = *streetIt;
      this->addAgent(destinationId, streetId);
      auto& pAgent{this->m_agents.back()};
      pAgent->setStreetId();
      pAgent->setSpeed(this->m_speedFunction(*street));
      pAgent->setFreeTime(this->time_step() +
                          std::ceil(street->length() / pAgent->speed()));
      street->addAgent(std::move(pAgent), this->time_step());
      this->m_agents.pop_back();
    }
  }

  void FirstOrderDynamics::addAgent(std::optional<Id> optDestinationId,
                                    std::optional<Id> optSrcStreetId) {
    std::unique_ptr<Agent> pAgent;
    if (!optSrcStreetId.has_value()) {
      pAgent =
          std::make_unique<Agent>(m_nInsertedAgents, this->time_step(), optDestinationId);
    } else {
      auto const& srcStreet{this->graph().edge(*optSrcStreetId)};
      pAgent = std::make_unique<Agent>(m_nInsertedAgents,
                                       this->time_step(),
                                       optDestinationId,
                                       srcStreet.source()->id());
      pAgent->setSrcStreetId(srcStreet.id());
      pAgent->setNextStreetId(srcStreet.id());
    }
    m_assignIntelligence(pAgent);
    addAgent(std::move(pAgent));
  }

  void FirstOrderDynamics::addAgent(std::unique_ptr<Agent> pAgent) {
    m_bIntelligentAgents = m_bIntelligentAgents || pAgent->isIntelligent();
    m_agents.push_back(std::move(pAgent));
    if (m_nAddedAgents == m_nInsertedAgents) {
      ++m_nAddedAgents;
    }
    ++m_nAgents;
    ++m_nInsertedAgents;
    auto const& optNextStreetId{m_agents.back()->nextStreetId()};
    if (optNextStreetId.has_value()) {
      auto [it, bInserted] = m_originCounts.insert({*optNextStreetId, 1});
      if (!bInserted) {
        ++it->second;
      }
    }
    spdlog::trace("Added {}", *m_agents.back());
  }

  void FirstOrderDynamics::addAgents(std::size_t const nAgents,
                                     AgentInsertionMethod const mode) {
    // Destinations which cannot be reached are removed before any is picked
    m_ensureRouter();
    switch (mode) {
      case AgentInsertionMethod::RANDOM:
        this->m_addAgentsRandom<true>(nAgents);
        break;
      case AgentInsertionMethod::RANDOM_WEIGHTED_ORIGIN:
        this->m_addAgentsRandom<false>(nAgents);
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
            "\"RANDOM\", \"RANDOM_WEIGHTED_ORIGIN\", \"ODS\", \"RANDOM_ODS\", "
            "\"CONDITIONAL_RANDOM_ODS\" and \"UNIFORM\"");
    }
  }

  StepDataResult FirstOrderDynamics::evolve(StepDataRequest const& dataRequest) {
    m_ensureRouter();
    StepDataResult stepData;
    stepData.timeStep = this->time_step();
    auto const n_threads{std::max<std::size_t>(1, this->concurrency())};
    // Sums for the average stats, accumulated per thread and merged after the parallel loop
    struct StepStats {
      double speed{0.}, speed2{0.}, travelTime{0.}, density{0.}, density2{0.}, queue{0.};
      std::size_t nValidEdges{0};
    };
    tbb::combinable<StepStats> localStats;
    bool const bComputeStats = dataRequest.saveAverageStats ||
                               dataRequest.saveStreetData || dataRequest.saveTravelData ||
                               dataRequest.saveAgentData;
    tbb::concurrent_map<Id, StreetDataRecord> streetDataRecords;

    spdlog::debug("Init evolve at time {}", this->time_step());
    // move the first agent of each street queue, if possible, putting it in the next node
    bool const bUpdateData =
        m_dataUpdatePeriod.has_value() && this->time_step() % *m_dataUpdatePeriod == 0;
    auto const numNodes{this->graph().nNodes()};
    auto const numEdges{this->graph().nEdges()};
    spdlog::debug("Init evolving streets at time {}", this->time_step());
    const auto grainSize = std::max<std::size_t>(1, numNodes / (n_threads * 8));
    this->m_taskArena.execute([&] {
      tbb::parallel_for(
          tbb::blocked_range<std::size_t>(0, numNodes, grainSize),
          [&](const tbb::blocked_range<std::size_t>& range) {
            auto& stats = localStats.local();
            // Looked up once per task: draws then cost as much as with a plain engine
            auto& rng = this->m_threadGenerators.local();
            for (std::size_t i = range.begin(); i != range.end(); ++i) {
              auto* pNode = &this->graph().node(m_nodeIndices[i]);
              for (auto* pStreet : pNode->ingoingEdges()) {
                auto const inEdgeId{pStreet->id()};
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
                  auto const itQueues{m_queuesAtTrafficLights.find(inEdgeId)};
                  if (itQueues == m_queuesAtTrafficLights.end()) {
                    spdlog::debug(
                        "Street {} is green in no phase of traffic light {}: skipping "
                        "queue-data collection for it.",
                        inEdgeId,
                        pNode->id());
                  } else {
                    for (auto& [direction, value] : itQueues->second) {
                      value += pStreet->nExitingAgents(direction, true);
                    }
                  }
                }
                m_evolveStreet(pStreet, pNode, rng);
                if (bComputeStats) {
                  auto const& density{pStreet->density<false>() * 1e3};
                  auto const& queueLength{pStreet->nExitingAgents()};

                  auto const speedMeasure = pStreet->meanSpeed<true>();
                  if (speedMeasure.is_valid) {
                    auto const speed = speedMeasure.mean * MS_TO_KMH;  // to kph
                    auto const speed_std = speedMeasure.std * MS_TO_KMH;
                    if (dataRequest.saveAverageStats) {
                      stats.speed += speed;
                      stats.speed2 += speed * speed + speed_std * speed_std;
                      stats.travelTime += pStreet->length() / speedMeasure.mean;
                      ++stats.nValidEdges;
                    }
                  }
                  if (dataRequest.saveAverageStats) {
                    stats.density += density;
                    stats.density2 += density * density;
                    stats.queue += queueLength;
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
                      record.avgSpeed = speedMeasure.mean * MS_TO_KMH;  // to kph
                      record.stdSpeed = speedMeasure.std * MS_TO_KMH;
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
    if (!m_agentsToReinsert.empty()) {
      // Sort by id, so that the insertion order does not depend on thread scheduling.
      // Sorted in place: the parallel section is over, so nothing else is appending to it
      std::ranges::sort(m_agentsToReinsert, {}, &Agent::id);
      for (auto& pAgent : m_agentsToReinsert) {
        this->addAgent(std::move(pAgent));
      }
      m_agentsToReinsert.clear();
    }
    spdlog::debug("Init evolving nodes at time {}", this->time_step());
    // Move transport capacity agents from each node
    this->m_taskArena.execute([&] {
      tbb::parallel_for(
          tbb::blocked_range<std::size_t>(0, numNodes, grainSize),
          [&](const tbb::blocked_range<std::size_t>& range) {
            for (std::size_t i = range.begin(); i != range.end(); ++i) {
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
    if (m_bIntelligentAgents) {
      // The traffic the intelligent agents plan on: no street changes until the next step
      this->m_taskArena.execute([this] { m_router.updateCurrentWeights(); });
    }
    spdlog::debug("Init evolving agents at time {}", this->time_step());
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
        auto const totals = localStats.combine([](StepStats a, StepStats const& b) {
          a.speed += b.speed;
          a.speed2 += b.speed2;
          a.travelTime += b.travelTime;
          a.density += b.density;
          a.density2 += b.density2;
          a.queue += b.queue;
          a.nValidEdges += b.nValidEdges;
          return a;
        });
        AverageStatsRecord averageStats;
        averageStats.nValidEdges = totals.nValidEdges;
        auto const edgeCount = static_cast<double>(numEdges);
        if (averageStats.nValidEdges > 0) {
          averageStats.meanSpeed = totals.speed / averageStats.nValidEdges;
          averageStats.stdSpeed =
              std::sqrt(std::max(0.0,
                                 totals.speed2 / averageStats.nValidEdges -
                                     averageStats.meanSpeed * averageStats.meanSpeed));
          averageStats.meanTravelTime = totals.travelTime / averageStats.nValidEdges;
        }
        if (edgeCount > 0.) {
          averageStats.meanDensity = totals.density / edgeCount;
          averageStats.stdDensity = std::sqrt(
              std::max(0.0,
                       totals.density2 / edgeCount -
                           averageStats.meanDensity * averageStats.meanDensity));
          averageStats.meanQueueLength = totals.queue / edgeCount;
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
      // Only the first `remainder` phases of the ordering are used
      std::vector<std::size_t> order(nPhases);
      std::iota(order.begin(), order.end(), 0u);
      std::ranges::partial_sort(order,
                                order.begin() + std::min<std::size_t>(remainder, nPhases),
                                std::ranges::greater{},
                                [&fractionals](auto const i) { return fractionals[i]; });
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
          std::ranges::sort(donors,
                            std::ranges::greater{},
                            [&newDurations](auto const i) { return newDurations[i]; });
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
        for (auto const* pStreet : pNode->ingoingEdges()) {
          if (!pStreet->source()->isTrafficLight()) {
            continue;
          }
          density += pStreet->density<true>();
          ++n;
        }
        if (n == 0.) {
          continue;
        }
        density /= n;
        densities[nodeId] = density;
      }
      // Sort densities map from big to small values
      std::vector<std::pair<Id, double>> sortedDensities(densities.begin(),
                                                         densities.end());

      // Sort by density descending
      std::ranges::sort(
          sortedDensities, std::ranges::greater{}, &std::pair<Id, double>::second);
      std::unordered_set<Id> optimizedNodes;

      for (auto const& [nodeId, density] : sortedDensities) {
        for (auto* pStreet : this->graph().node(nodeId).ingoingEdges()) {
          auto const sourceId{pStreet->source()->id()};
          if (!densities.contains(sourceId) || optimizedNodes.contains(sourceId)) {
            continue;
          }
          auto const& neighbourDensity{densities.at(sourceId)};
          if (neighbourDensity < threshold * density) {
            continue;
          }
          // Try to green-wave the situation
          auto& tl{dynamic_cast<TrafficLight&>(*pStreet->source())};
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
    Measurement<double> const travelTimes{
        m_travelDTs | std::views::transform(&TravelRecord::travelTime)};
    if (clearData) {
      m_travelDTs.clear();
    }
    return travelTimes;
  }
  Measurement<double> FirstOrderDynamics::meanTravelDistance(bool clearData) {
    Measurement<double> const travelDistances{
        m_travelDTs | std::views::transform(&TravelRecord::distance)};
    if (clearData) {
      m_travelDTs.clear();
    }
    return travelDistances;
  }
  Measurement<double> FirstOrderDynamics::meanTravelSpeed(bool clearData) {
    Measurement<double> const travelSpeeds{
        m_travelDTs | std::views::transform([](TravelRecord const& travelDT) {
          return travelDT.distance / travelDT.travelTime;
        })};
    if (clearData) {
      m_travelDTs.clear();
    }
    return travelSpeeds;
  }
  std::unordered_map<Id, std::unordered_map<Id, double>> const
  FirstOrderDynamics::normalizedTurnCounts() const noexcept {
    std::unordered_map<Id, std::unordered_map<Id, double>> normalizedTurnCounts;
    for (auto const& [fromId, map] : m_turnCounts) {
      auto const counts{map | std::views::values};
      auto const sum{static_cast<double>(
          std::accumulate(counts.begin(), counts.end(), std::size_t{0}))};
      for (auto const& [toId, count] : map) {
        normalizedTurnCounts[fromId][toId] =
            sum == 0. ? 0. : static_cast<double>(count) / sum;
      }
    }
    return normalizedTurnCounts;
  }

  tbb::concurrent_unordered_map<Id, std::size_t> FirstOrderDynamics::m_countsByMode(
      tbb::concurrent_unordered_map<Id, std::size_t> streetCounts,
      bool const bDestination) const {
    if (m_mode == DynamicsMode::EDGES) {
      return streetCounts;
    }
    // Origins are counted on the street the agent starts with, arrivals on the street at
    // whose end it stops
    tbb::concurrent_unordered_map<Id, std::size_t> nodeCounts;
    for (auto const& [streetId, count] : streetCounts) {
      auto const& street{this->graph().edge(streetId)};
      nodeCounts[(bDestination ? street.target() : street.source())->id()] += count;
    }
    return nodeCounts;
  }
  tbb::concurrent_unordered_map<Id, std::size_t> FirstOrderDynamics::originCounts(
      bool const bReset) {
    if (!bReset) {
      return m_countsByMode(m_originCounts, false);
    }
    auto tempCounts{std::move(m_originCounts)};
    m_originCounts.clear();
    return m_countsByMode(std::move(tempCounts), false);
  }
  tbb::concurrent_unordered_map<Id, std::size_t> FirstOrderDynamics::destinationCounts(
      bool const bReset) {
    if (!bReset) {
      return m_countsByMode(m_destinationCounts, true);
    }
    auto tempCounts{std::move(m_destinationCounts)};
    m_destinationCounts.clear();
    return m_countsByMode(std::move(tempCounts), true);
  }

  Measurement<double> FirstOrderDynamics::streetMeanFlow() const {
    std::vector<double> flows;
    flows.reserve(this->graph().nEdges());
    for (const auto& [streetId, pStreet] : this->graph().edges()) {
      auto const speedMeasure = pStreet->meanSpeed<false>();
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
      auto const speedMeasure = pStreet->meanSpeed<false>();
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

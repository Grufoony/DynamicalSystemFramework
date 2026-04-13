#include "RoadNetwork.hpp"
#include "../geometry/Point.hpp"
#include "../geometry/PolyLine.hpp"

#include <algorithm>
#include <ranges>

#include <csv.hpp>
#include <simdjson.h>
#include <tbb/parallel_for_each.h>

// Default traffic light cycle duration in seconds
constexpr dsf::Delay TRAFFICLIGHT_DEFAULT_CYCLE = 90;

namespace dsf::mobility {
  void RoadNetwork::m_updateMaxAgentCapacity() {
    m_capacity = 0;
    for (auto const& [_, pStreet] : this->edges()) {
      m_capacity += pStreet->capacity();
    }
  }

  void RoadNetwork::m_csvEdgesImporter(const std::string& fileName,
                                       const char separator) {
    csv::CSVFormat format;
    format.delimiter(separator);
    csv::CSVReader reader(fileName, format);

    auto const& colNames = reader.get_col_names();
    bool const bHasGeometry =
        (std::find(colNames.begin(), colNames.end(), "geometry") != colNames.end());
    if (!bHasGeometry) {
      spdlog::warn(
          "No geometry column found in the CSV file. Streets will be imported without "
          "geometry.");
    }
    bool const bHasLanes =
        (std::find(colNames.begin(), colNames.end(), "nlanes") != colNames.end());
    bool const bHasCoilcode =
        (std::find(colNames.begin(), colNames.end(), "coilcode") != colNames.end());
    bool const bHasCustomWeight =
        (std::find(colNames.begin(), colNames.end(), "customWeight") != colNames.end());
    bool const bHasPriority =
        (std::find(colNames.begin(), colNames.end(), "priority") != colNames.end());

    for (auto& row : reader) {
      auto const sourceId = row["source"].get<Id>();
      auto const targetId = row["target"].get<Id>();
      if (sourceId == targetId) {
        spdlog::warn("Skipping self-loop edge {}->{}", sourceId, targetId);
        continue;
      }
      auto const streetId = row["id"].get<Id>();
      auto const dLength = row["length"].get<double>();
      auto const name = row["name"].get<std::string>();
      auto strType = row["type"].get<std::string>();
      geometry::PolyLine polyline;
      if (bHasGeometry) {
        polyline = geometry::PolyLine(row["geometry"].get<std::string>());
      }

      auto iLanes = 1;
      if (bHasLanes) {
        try {
          iLanes = row["nlanes"].get<int>();
        } catch (...) {
          spdlog::warn("Invalid number of lanes for edge {}->{}. Defaulting to 1 lane.",
                       sourceId,
                       targetId);
          iLanes = 1;
        }
      }

      double dMaxSpeed = 30.;  // Default to 30 km/h
      try {
        dMaxSpeed = row["maxspeed"].get<double>();
      } catch (...) {
        spdlog::warn("Invalid maxspeed provided for edge {}->{}. Defaulting to 30 km/h.",
                     sourceId,
                     targetId);
      }
      dMaxSpeed /= 3.6;  // Convert to m/s

      addStreet(Street(streetId,
                       std::make_pair(sourceId, targetId),
                       dLength,
                       dMaxSpeed,
                       iLanes,
                       name,
                       polyline));

      if (!strType.empty()) {
        std::transform(
            strType.begin(), strType.end(), strType.begin(), [](unsigned char c) {
              return std::tolower(c);
            });
        if (strType.find("motorway") != std::string::npos) {
          edge(streetId).setRoadType(RoadType::HIGHWAY);
        } else if (strType.find("primary") != std::string::npos) {
          edge(streetId).setRoadType(RoadType::PRIMARY);
        } else if (strType.find("secondary") != std::string::npos) {
          edge(streetId).setRoadType(RoadType::SECONDARY);
        } else if (strType.find("tertiary") != std::string::npos) {
          edge(streetId).setRoadType(RoadType::TERTIARY);
        } else if (strType.find("residential") != std::string::npos) {
          edge(streetId).setRoadType(RoadType::RESIDENTIAL);
        }
      }

      if (bHasPriority) {
        try {
          if (row["priority"].get<bool>()) {
            edge(streetId).setPriority();
          }
        } catch (...) {
          spdlog::warn("Invalid priority for edge {}.", streetId);
        }
      }

      if (bHasCoilcode) {
        auto strCoilCode = row["coilcode"].get<std::string>();
        // Make this lowercase
        std::transform(strCoilCode.begin(),
                       strCoilCode.end(),
                       strCoilCode.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        // Do not warn if the coilcode contains null or nan
        if (!strCoilCode.empty() && strCoilCode != "null" && strCoilCode != "nan") {
          addCoil(streetId, strCoilCode);
        }
      }
      if (bHasCustomWeight) {
        try {
          edge(streetId).setWeight(row["customWeight"].get<double>());
        } catch (...) {
          spdlog::warn("Invalid custom weight for {}", edge(streetId));
        }
      }
    }
    this->m_nodes.rehash(0);
    this->m_edges.rehash(0);
    // Parse forbidden turns
    // for (auto const& [streetId, forbiddenTurns] : mapForbiddenTurns) {
    //   auto* pStreet{&edge(streetId)};
    //   std::istringstream iss{forbiddenTurns};
    //   std::string pair;
    //   while (std::getline(iss, pair, ',')) {
    //     // Decompose pair = sourceId-targetId
    //     std::istringstream pairStream(pair);
    //     std::string strSourceId, strTargetId;
    //     std::getline(pairStream, strSourceId, '-');
    //     // targetId is the remaining part
    //     std::getline(pairStream, strTargetId);

    //     Id const sourceId{std::stoul(strSourceId)};
    //     Id const targetId{std::stoul(strTargetId)};

    //     pStreet->addForbiddenTurn(edge(sourceId, targetId).id());
    //   }
    // }
  }
  void RoadNetwork::m_csvNodePropertiesImporter(const std::string& fileName,
                                                const char separator) {
    csv::CSVFormat format;
    format.delimiter(separator);
    csv::CSVReader reader(fileName, format);

    for (auto& row : reader) {
      auto const nodeId = row["id"].get<Id>();
      if (m_nodes.find(nodeId) == m_nodes.end()) {
        spdlog::warn("Node {} not found in the network. Skipping properties import.",
                     nodeId);
        continue;
      }
      auto strType = row["type"].get<std::string>();
      std::transform(
          strType.begin(), strType.end(), strType.begin(), [](unsigned char c) {
            return std::tolower(c);
          });
      if (strType.find("traffic_signals") != std::string::npos) {
        makeTrafficLight(nodeId, TRAFFICLIGHT_DEFAULT_CYCLE);
      } else if (strType.find("roundabout") != std::string::npos) {
        makeRoundabout(nodeId);
      }
      auto const& strGeometry = row["geometry"].get<std::string>();
      if (!strGeometry.empty()) {
        auto const point = geometry::Point(strGeometry);
        auto& nodeRef{node(nodeId)};
        // Assign geometry or check if these geometry match the existing ones
        if (!nodeRef.geometry().has_value()) {
          nodeRef.setGeometry(point);
        } else {
          auto const& [oldLon, oldLat] = nodeRef.geometry().value();
          auto const& [newLon, newLat] = point;
          if (std::abs(oldLat - newLat) > 1e-4 || std::abs(oldLon - newLon) > 1e-4) {
            spdlog::error(
                "Node {} geometry from properties file ({}, {}) do not match existing "
                "geometry ({}, {}). Keeping existing geometry.",
                nodeId,
                newLat,
                newLon,
                oldLat,
                oldLon);
          }
        }
      }
    }
  }
  void RoadNetwork::m_jsonEdgesImporter(std::ifstream& file) {
    // Read the file into a string
    std::string json_str((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
    simdjson::dom::parser parser;
    simdjson::dom::element root;
    auto error = parser.parse(simdjson::padded_string(json_str)).get(root);
    if (error) {
      throw std::runtime_error("Failed to parse JSON: " +
                               std::string(simdjson::error_message(error)));
    }

    for (auto feature : root["features"]) {
      auto edge_properties = feature["properties"];

      auto const& src_node_id = static_cast<Id>(edge_properties["source"].get_uint64());
      auto const& dst_node_id = static_cast<Id>(edge_properties["target"].get_uint64());
      if (src_node_id == dst_node_id) {
        spdlog::warn("Skipping self-loop edge {}->{}", src_node_id, dst_node_id);
        continue;
      }

      geometry::PolyLine geometry;
      for (auto const& coord : feature["geometry"]["coordinates"]) {
        auto const& lat = coord.at(1);
        auto const& lon = coord.at(0);
        geometry.emplace_back(lon, lat);
      }
      auto const& edge_id = static_cast<Id>(edge_properties["id"].get_uint64());
      auto const& edge_length =
          static_cast<double>(edge_properties["length"].get_double());

      std::string strType{""};
      switch (edge_properties["type"].type()) {
        case simdjson::dom::element_type::STRING:
          strType =
              static_cast<std::string>(edge_properties["type"].get_string().value());
          break;
        case simdjson::dom::element_type::ARRAY: {
          auto type_array = edge_properties["type"].get_array().value();
          for (auto const& type_elem : type_array) {
            if (!type_elem.is_string()) {
              spdlog::warn("Edge {} type array contains non-string element", edge_id);
              continue;
            }
            strType += static_cast<std::string>(type_elem.get_string().value()) + '|';
          }
          if (!strType.empty()) {
            strType.pop_back();  // Remove last '|'
          }
          break;
        }
        default:
          spdlog::warn("Edge {} type is of unexpected type", edge_id);
          break;
      }

      // Robust extraction for maxspeed
      double edge_maxspeed = 30.0;
      if (!edge_properties["maxspeed"].is_null()) {
        auto maxspeed_val = edge_properties["maxspeed"];
        if (maxspeed_val.is_string()) {
          try {
            edge_maxspeed = std::stod(std::string(maxspeed_val.get_string().value()));
          } catch (...) {
            edge_maxspeed = 30.0;
          }
        } else if (maxspeed_val.is_number()) {
          edge_maxspeed = maxspeed_val.get_double();
        }
      }
      edge_maxspeed /= 3.6;

      // Robust extraction for lanes
      auto edge_lanes{1u};
      if (!edge_properties["nlanes"].is_null()) {
        auto lanes_val = edge_properties["nlanes"];
        if (lanes_val.is_number()) {
          edge_lanes = lanes_val.get_uint64();
        } else if (lanes_val.is_string()) {
          try {
            edge_lanes = std::stoul(std::string(lanes_val.get_string().value()));
          } catch (...) {
            edge_lanes = 1;
          }
        }
      }

      // Robust extraction for name
      std::string name = "";
      if (!edge_properties["name"].is_null() && edge_properties["name"].is_string()) {
        name = std::string(edge_properties["name"].get_string().value());
      }

      addStreet(Street(edge_id,
                       std::make_pair(src_node_id, dst_node_id),
                       edge_length,
                       edge_maxspeed,
                       edge_lanes,
                       name,
                       geometry));
      if (!strType.empty()) {
        std::transform(
            strType.begin(), strType.end(), strType.begin(), [](unsigned char c) {
              return std::tolower(c);
            });
        if (strType.find("motorway") != std::string::npos) {
          edge(edge_id).setRoadType(RoadType::HIGHWAY);
        } else if (strType.find("primary") != std::string::npos) {
          edge(edge_id).setRoadType(RoadType::PRIMARY);
        } else if (strType.find("secondary") != std::string::npos) {
          edge(edge_id).setRoadType(RoadType::SECONDARY);
        } else if (strType.find("tertiary") != std::string::npos) {
          edge(edge_id).setRoadType(RoadType::TERTIARY);
        } else if (strType.find("residential") != std::string::npos) {
          edge(edge_id).setRoadType(RoadType::RESIDENTIAL);
        }
      }
      // Check if there is coilcode property
      if (!edge_properties.at_key("coilcode").error()) {
        auto const& epCoilCode = edge_properties["coilcode"];
        if (epCoilCode.is_string()) {
          std::string strCoilCode{epCoilCode.get_string().value()};
          addCoil(edge_id, strCoilCode);
        } else if (epCoilCode.is_uint64()) {
          std::string strCoilCode = std::to_string(epCoilCode.get_uint64());
          addCoil(edge_id, strCoilCode);
        } else if (epCoilCode.is_int64()) {
          std::string strCoilCode = std::to_string(epCoilCode.get_int64());
          addCoil(edge_id, strCoilCode);
        } else {
          spdlog::warn("Invalid coilcode for edge {}, adding default", edge_id);
          addCoil(edge_id);
        }
      }
      // Check if there is custom weight property
      if (!edge_properties.at_key("customWeight").error()) {
        auto const& epCustomWeight = edge_properties["customWeight"];
        if (epCustomWeight.is_number()) {
          edge(edge_id).setWeight(epCustomWeight.get_double());
        } else {
          spdlog::warn("Invalid custom weight for edge {}, keeping default", edge_id);
        }
      }
      // Check if there is priority property
      if (!edge_properties.at_key("priority").error()) {
        auto const& epPriority = edge_properties["priority"];
        if (epPriority.is_bool()) {
          if (epPriority.get_bool()) {
            edge(edge_id).setPriority();
          }
        } else {
          spdlog::warn("Invalid priority for edge {}, keeping default", edge_id);
        }
      }
    }
    this->m_nodes.rehash(0);
    this->m_edges.rehash(0);
  }

  std::size_t RoadNetwork::nCoils() const {
    return std::count_if(m_edges.cbegin(), m_edges.cend(), [](auto const& pair) {
      return pair.second->hasCoil();
    });
  }

  std::size_t RoadNetwork::nIntersections() const {
    return std::count_if(m_nodes.cbegin(), m_nodes.cend(), [](auto const& pair) {
      return pair.second->isIntersection();
    });
  }
  std::size_t RoadNetwork::nRoundabouts() const {
    return std::count_if(m_nodes.cbegin(), m_nodes.cend(), [](auto const& pair) {
      return pair.second->isRoundabout();
    });
  }
  std::size_t RoadNetwork::nTrafficLights() const {
    return std::count_if(m_nodes.cbegin(), m_nodes.cend(), [](auto const& pair) {
      return pair.second->isTrafficLight();
    });
  }

  void RoadNetwork::autoInitTrafficLights(double const mainRoadPercentage) {
    tbb::parallel_for_each(
        m_nodes.begin(), m_nodes.end(), [this, mainRoadPercentage](auto& pair) {
          auto& pNode = pair.second;
          if (!pNode->isTrafficLight()) {
            return;
          }
          auto& tl = static_cast<TrafficLight&>(*pNode);
          if (!tl.streetPriorities().empty() || !tl.cycles().empty()) {
            return;
          }
          auto const& inNeighbours = pNode->ingoingEdges();
          std::map<Id, int, std::greater<int>> capacities;
          std::unordered_map<Id, double> streetAngles;
          std::unordered_map<Id, double> maxSpeeds;
          std::unordered_map<Id, int> nLanes;
          std::unordered_map<Id, std::string> streetNames;
          double higherSpeed{0.}, lowerSpeed{std::numeric_limits<double>::max()};
          int higherNLanes{0}, lowerNLanes{std::numeric_limits<int>::max()};
          if (inNeighbours.size() < 3) {
            spdlog::warn("Not enough in neighbours {} for Traffic Light {}",
                         inNeighbours.size(),
                         pNode->id());
            // Replace with a normal intersection, preserving node properties (edges, etc.)
            pNode = std::make_unique<Intersection>(*pNode);
            return;
          }
          for (auto const& edgeId : inNeighbours) {
            auto* pStreet{&edge(edgeId)};

            double const speed{pStreet->maxSpeed()};
            int const nLan{pStreet->nLanes()};
            auto const cap{pStreet->capacity()};
            capacities.emplace(pStreet->id(), cap);
            auto angle{pStreet->angle()};
            if (angle < 0.) {
              angle += 2 * std::numbers::pi;
            }
            streetAngles.emplace(pStreet->id(), angle);

            maxSpeeds.emplace(pStreet->id(), speed);
            nLanes.emplace(pStreet->id(), nLan);
            streetNames.emplace(pStreet->id(), pStreet->name());

            higherSpeed = std::max(higherSpeed, speed);
            lowerSpeed = std::min(lowerSpeed, speed);

            higherNLanes = std::max(higherNLanes, nLan);
            lowerNLanes = std::min(lowerNLanes, nLan);

            if (pStreet->hasPriority()) {
              tl.addStreetPriority(pStreet->id());
            }
          }

          if (tl.streetPriorities().empty()) {
            /*************************************************************
         * 1. Check for street names with multiple occurrences
         * ***********************************************************/
            std::unordered_map<std::string, int> counts;
            for (auto const& [streetId, name] : streetNames) {
              if (name.empty()) {
                // Ignore empty names
                return;
              }
              if (!counts.contains(name)) {
                counts[name] = 1;
              } else {
                ++counts.at(name);
              }
            }
            // Check if spdlog is in debug mode
            if (spdlog::get_level() <= spdlog::level::debug) {
              for (auto const& [name, count] : counts) {
                spdlog::debug("Street name {} has {} occurrences", name, count);
              }
            }
            for (auto const& [streetId, name] : streetNames) {
              if (!name.empty() && counts.at(name) > 1) {
                tl.addStreetPriority(streetId);
              }
            }
          }
          if (tl.streetPriorities().empty() && higherSpeed != lowerSpeed) {
            /*************************************************************
         * 2. Check for street names with same max speed
         * ***********************************************************/
            for (auto const& [sid, speed] : maxSpeeds) {
              if (speed == higherSpeed) {
                tl.addStreetPriority(sid);
              }
            }
          }
          if (tl.streetPriorities().empty() && higherNLanes != lowerNLanes) {
            /*************************************************************
         * 2. Check for street names with same number of lanes
         * ***********************************************************/
            for (auto const& [sid, nLan] : nLanes) {
              if (nLan == higherNLanes) {
                tl.addStreetPriority(sid);
              }
            }
          }
          if (tl.streetPriorities().empty()) {
            /*************************************************************
         * 3. Check for streets with opposite angles
         * ***********************************************************/
            std::vector<std::pair<Id, double>> sortedAngles;
            std::copy(streetAngles.begin(),
                      streetAngles.end(),
                      std::back_inserter(sortedAngles));
            std::sort(sortedAngles.begin(),
                      sortedAngles.end(),
                      [](auto const& a, auto const& b) { return a.second < b.second; });
            streetAngles.clear();
            for (auto const& [streetId, angle] : sortedAngles) {
              streetAngles.emplace(streetId, angle);
            }

            auto const& streetId = streetAngles.begin()->first;
            auto const& angle = streetAngles.begin()->second;
            for (auto const& [streetId2, angle2] : streetAngles) {
              if (std::abs(angle - angle2) > 0.75 * std::numbers::pi) {
                tl.addStreetPriority(streetId);
                tl.addStreetPriority(streetId2);
                break;
              }
            }
          }
          if (tl.streetPriorities().empty() || tl.streetPriorities().size() != 2) {
            spdlog::warn("Failed to auto-init Traffic Light {} - going random",
                         pNode->id());
            // Assign first and third keys of capacity map
            auto it = capacities.begin();
            auto const& firstKey = it->first;
            ++it;
            ++it;
            auto const& thirdKey = it->first;
            tl.addStreetPriority(firstKey);
            tl.addStreetPriority(thirdKey);
          }

          // Assign cycles
          std::pair<Delay, Delay> greenTimes;
          {
            auto const mainGreenTime{
                static_cast<Delay>(mainRoadPercentage * tl.cycleTime())};
            auto const secondaryGreenTime{
                static_cast<Delay>(tl.cycleTime() - mainGreenTime)};
            greenTimes = std::make_pair(mainGreenTime, secondaryGreenTime);
          }
          std::for_each(inNeighbours.begin(), inNeighbours.end(), [&](auto const& edgeId) {
            auto const streetId{this->edge(edgeId).id()};
            auto const nLane{nLanes.at(streetId)};
            Delay greenTime{greenTimes.first};
            Delay phase{0};
            if (!tl.streetPriorities().contains(streetId)) {
              phase = greenTime;
              greenTime = greenTimes.second;
            }
            spdlog::debug("Setting cycle for street {} with green time {} and phase {}",
                          streetId,
                          greenTime,
                          phase);
            switch (nLane) {
              case 3:
                tl.setCycle(
                    streetId,
                    dsf::Direction::RIGHTANDSTRAIGHT,
                    TrafficLightCycle{static_cast<Delay>(greenTime * 2. / 3), phase});
                tl.setCycle(streetId,
                            dsf::Direction::LEFT,
                            TrafficLightCycle{
                                static_cast<Delay>(greenTime / 3.),
                                static_cast<Delay>(
                                    phase + static_cast<Delay>(greenTime * 2. / 3))});
                break;
              default:
                tl.setCycle(
                    streetId, dsf::Direction::ANY, TrafficLightCycle{greenTime, phase});
                break;
            }
          });
        });
  }
  void RoadNetwork::autoMapStreetLanes() {
    auto const& nodes = this->nodes();
    std::for_each(nodes.cbegin(), nodes.cend(), [this](auto const& pair) {
      auto const& pNode{pair.second};
      auto const& inNeighbours{pNode->ingoingEdges()};
      auto const& outNeighbours{pNode->outgoingEdges()};
      double maxEstimatedFlow{0.0};
      std::for_each(inNeighbours.cbegin(),
                    inNeighbours.cend(),
                    [this, &maxEstimatedFlow](auto const& edgeId) {
                      auto* pStreet{&this->edge(edgeId)};
                      auto const estFlow{pStreet->maxSpeed() * pStreet->nLanes()};
                      maxEstimatedFlow = std::max(maxEstimatedFlow, estFlow);
                    });
      std::for_each(outNeighbours.cbegin(),
                    outNeighbours.cend(),
                    [this, &maxEstimatedFlow](auto const& edgeId) {
                      auto* pStreet{&this->edge(edgeId)};
                      auto const estFlow{pStreet->maxSpeed() * pStreet->nLanes()};
                      maxEstimatedFlow = std::max(maxEstimatedFlow, estFlow);
                    });
      std::for_each(
          inNeighbours.cbegin(),
          inNeighbours.cend(),
          [this, &pNode, &outNeighbours, &maxEstimatedFlow](auto const& edgeId) {
            auto* pInStreet{&this->edge(edgeId)};
            auto const nLanes{pInStreet->nLanes()};
            if (nLanes == 1) {
              return;
            }
            std::multiset<Direction> allowedTurns;
            std::for_each(
                outNeighbours.cbegin(),
                outNeighbours.cend(),
                [this, &pInStreet, &allowedTurns, &maxEstimatedFlow](auto const& edgeId) {
                  auto const* pOutStreet{&this->edge(edgeId)};
                  if (pOutStreet->target() == pInStreet->source() ||
                      pInStreet->forbiddenTurns().contains(pOutStreet->id())) {
                    return;
                  }
                  auto const deltaAngle{pOutStreet->deltaAngle(pInStreet->angle())};
                  auto const& outOppositeStreet{
                      this->street(pOutStreet->target(), pOutStreet->source())};
                  if (!outOppositeStreet) {
                    return;
                  }
                  // Actually going straight means remain on the same road, thus...
                  auto const inEstFlow{pInStreet->maxSpeed() * pInStreet->nLanes()};
                  auto const outEstFlow{outOppositeStreet->maxSpeed() *
                                        outOppositeStreet->nLanes()};
                  if (((inEstFlow == maxEstimatedFlow) ==
                       (outEstFlow == maxEstimatedFlow)) &&
                      !allowedTurns.contains(Direction::STRAIGHT)) {
                    spdlog::debug("Street {} prioritized STRAIGHT", pInStreet->id());
                    if (allowedTurns.contains(Direction::STRAIGHT) &&
                        !allowedTurns.contains(Direction::RIGHT)) {
                      allowedTurns.emplace(Direction::RIGHT);
                    } else {
                      allowedTurns.emplace(Direction::STRAIGHT);
                    }
                    // if (!allowedTurns.contains(Direction::STRAIGHT)) {
                    // allowedTurns.emplace(Direction::STRAIGHT);
                    // return;
                    // }
                  } else if (std::abs(deltaAngle) < std::numbers::pi) {
                    // Logger::debug(std::format("Angle in {} - angle out {}",
                    //                           pInStreet->angle(),
                    //                           pOutStreet->angle()));
                    // Logger::debug(std::format("Delta: {}", deltaAngle));
                    if (std::abs(deltaAngle) < std::numbers::pi / 8) {
                      spdlog::debug("Street {} -> {} can turn STRAIGHT",
                                    pInStreet->source(),
                                    pInStreet->target());
                      allowedTurns.emplace(Direction::STRAIGHT);
                    } else if (deltaAngle < 0.) {
                      spdlog::debug("Street {} -> {} can turn RIGHT",
                                    pInStreet->source(),
                                    pInStreet->target());
                      allowedTurns.emplace(Direction::RIGHT);
                    } else if (deltaAngle > 0.) {
                      spdlog::debug("Street {} -> {} can turn LEFT",
                                    pInStreet->source(),
                                    pInStreet->target());
                      allowedTurns.emplace(Direction::LEFT);
                    }
                  }
                });
            while (allowedTurns.size() < static_cast<size_t>(nLanes)) {
              if (allowedTurns.contains(Direction::STRAIGHT)) {
                allowedTurns.emplace(Direction::STRAIGHT);
              } else if (allowedTurns.contains(Direction::RIGHT)) {
                allowedTurns.emplace(Direction::RIGHT);
              } else if (allowedTurns.contains(Direction::LEFT)) {
                allowedTurns.emplace(Direction::LEFT);
              } else {
                allowedTurns.emplace(Direction::ANY);
              }
            }
            // If allowedTurns contains all RIGHT, STRAIGHT and LEFT, transform RIGHT into RIGHTANDSTRAIGHT
            if (allowedTurns.size() > static_cast<size_t>(nLanes)) {
              if (pNode->isTrafficLight()) {
                auto& tl = static_cast<TrafficLight&>(*pNode);
                auto const& cycles{tl.cycles()};
                if (cycles.contains(pInStreet->id())) {
                  if (cycles.size() == static_cast<size_t>(nLanes)) {
                    // Replace with the traffic light cycles
                    spdlog::debug("Using traffic light {} cycles for street {} -> {}",
                                  tl.id(),
                                  pInStreet->source(),
                                  pInStreet->target());
                    auto const& cycle{cycles.at(pInStreet->id())};
                    allowedTurns.clear();
                    for (auto const& [direction, cycle] : cycle) {
                      allowedTurns.emplace(direction);
                    }
                  } else if (cycles.at(pInStreet->id())
                                 .contains(Direction::LEFTANDSTRAIGHT)) {
                    allowedTurns.erase(Direction::LEFT);
                    allowedTurns.erase(Direction::STRAIGHT);
                    allowedTurns.emplace(Direction::LEFTANDSTRAIGHT);
                  } else if (cycles.at(pInStreet->id())
                                 .contains(Direction::RIGHTANDSTRAIGHT)) {
                    allowedTurns.erase(Direction::RIGHT);
                    allowedTurns.erase(Direction::STRAIGHT);
                    allowedTurns.emplace(Direction::RIGHTANDSTRAIGHT);
                  }
                }
              }
            }
            if (allowedTurns.size() > static_cast<size_t>(nLanes)) {
              // if one is duplicate, remove it
              std::set<Direction> uniqueDirections;
              std::copy(allowedTurns.begin(),
                        allowedTurns.end(),
                        std::inserter(uniqueDirections, uniqueDirections.begin()));
              allowedTurns.clear();
              std::copy(uniqueDirections.begin(),
                        uniqueDirections.end(),
                        std::inserter(allowedTurns, allowedTurns.begin()));
            }
            while (allowedTurns.size() < static_cast<size_t>(nLanes)) {
              if (allowedTurns.contains(Direction::STRAIGHT)) {
                allowedTurns.emplace(Direction::STRAIGHT);
              } else if (allowedTurns.contains(Direction::RIGHT)) {
                allowedTurns.emplace(Direction::RIGHT);
              } else if (allowedTurns.contains(Direction::LEFT)) {
                allowedTurns.emplace(Direction::LEFT);
              } else {
                allowedTurns.emplace(Direction::ANY);
              }
            }
            switch (nLanes) {
              case 1:
                // Leaving Direction::ANY for one lane streets is the less painful option
                break;
              case 2:
                if (allowedTurns.contains(Direction::STRAIGHT) &&
                    allowedTurns.contains(Direction::RIGHT) &&
                    allowedTurns.contains(Direction::LEFT)) {
                  if (pNode->isTrafficLight()) {
                    auto& tl = static_cast<TrafficLight&>(*pNode);
                    auto const& cycles{tl.cycles()};
                    if (cycles.contains(pInStreet->id())) {
                      auto const& cycle{cycles.at(pInStreet->id())};
                      if (cycle.contains(Direction::LEFTANDSTRAIGHT) &&
                          cycle.contains(Direction::RIGHT)) {
                        allowedTurns.erase(Direction::LEFT);
                        allowedTurns.erase(Direction::STRAIGHT);
                        allowedTurns.emplace(Direction::LEFTANDSTRAIGHT);
                        break;
                      }
                    }
                  }
                  allowedTurns.clear();
                  allowedTurns.emplace(Direction::RIGHTANDSTRAIGHT);
                  allowedTurns.emplace(Direction::LEFT);
                }
                if (allowedTurns.size() > 2) {
                  // Remove duplicates
                  std::set<Direction> uniqueDirections;
                  std::copy(allowedTurns.begin(),
                            allowedTurns.end(),
                            std::inserter(uniqueDirections, uniqueDirections.begin()));
                  allowedTurns.clear();
                  std::copy(uniqueDirections.begin(),
                            uniqueDirections.end(),
                            std::inserter(allowedTurns, allowedTurns.begin()));
                }
                [[fallthrough]];
              default:
                // Logger::info(std::format(
                //     "Street {}->{} with {} lanes and {} allowed turns",
                //     pInStreet->source(),
                //     pInStreet->target(),
                //     nLanes,
                //     allowedTurns.size()));
                assert(allowedTurns.size() == static_cast<size_t>(nLanes));
                // Logger::info(
                //     std::format("Street {}->{} with {} lanes and {} allowed turns",
                //                 pInStreet->source(),
                //                 pInStreet->target(),
                //                 nLanes,
                //                 allowedTurns.size()));
                std::vector<Direction> newMapping(nLanes);
                auto it{allowedTurns.cbegin()};
                for (size_t i{0}; i < allowedTurns.size(); ++i, ++it) {
                  newMapping[i] = *it;
                }
                // If the last one is RIGHTANDSTRAIGHT, move it in front
                if (newMapping.back() == Direction::RIGHTANDSTRAIGHT) {
                  std::rotate(
                      newMapping.rbegin(), newMapping.rbegin() + 1, newMapping.rend());
                }
                pInStreet->setLaneMapping(newMapping);
            }
          });
    });
  }

  void RoadNetwork::autoAssignRoadPriorities() {
    spdlog::debug("Auto-assigning road priorities...");
    tbb::parallel_for_each(m_nodes.cbegin(), m_nodes.cend(), [this](auto const& pair) {
      auto const& pNode{pair.second};
      auto const& inNeighbours{pNode->ingoingEdges()};
      // NOTE: std::multimap iterates keys in ascending order of RoadType.
      // RoadType is defined so that more important roads (e.g., HIGHWAY = 0,
      // PRIMARY = 1, SECONDARY = 2, ...) have smaller enum values. The logic
      // below relies on this ordering to consider higher-priority road types
      // first when selecting streets to mark as priority roads.
      std::multimap<RoadType, Id> types;
      for (auto const& edgeId : inNeighbours) {
        auto* pStreet{&this->edge(edgeId)};
        auto const roadType = pStreet->roadType();
        if (roadType.has_value()) {
          types.emplace(roadType.value(), pStreet->id());
        }
      }
      if (types.size() < 2) {
        return;
      }
      std::vector<Id> priorityRoads;
      // Find the first road type that has at least 2 streets
      for (auto it = types.begin(); it != types.end();) {
        auto const& currentType = it->first;
        auto const count = types.count(currentType);

        if (count == 2) {
          auto range = types.equal_range(currentType);
          for (auto rangeIt = range.first; rangeIt != range.second; ++rangeIt) {
            priorityRoads.push_back(rangeIt->second);
          }
          break;
        }

        // Move to the next different type
        it = types.upper_bound(currentType);
      }

      if (priorityRoads.size() < 2) {
        spdlog::warn("{}: unable to auto-assign road priorities", *pNode);
        return;
      }

      for (auto const& streetId : priorityRoads) {
        auto* pStreet{&this->edge(streetId)};
        pStreet->setPriority();
        spdlog::debug("Setting priority to street {}", pStreet->id());
      }
    });
    spdlog::debug("Done auto-assigning road priorities.");
  }

  void RoadNetwork::describe(std::ostream& os) const {
    os << "RoadNetwork with " << nNodes() << " nodes and " << nEdges()
       << " edges. Total capacity: " << m_capacity << " vehicles.\n"
       << "There are\n- " << nIntersections() << " intersections,\n- " << nTrafficLights()
       << " traffic lights,\n- " << nRoundabouts() << " roundabouts,\n- " << nCoils()
       << " coil sensors.\n";
  }

  void RoadNetwork::adjustNodeCapacities() {
    double value;
    for (auto const& [_, pNode] : nodes()) {
      value = 0.;
      for (auto const& edgeId : pNode->ingoingEdges()) {
        auto* pStreet{&this->edge(edgeId)};
        value += pStreet->nLanes() * pStreet->transportCapacity();
      }
      pNode->setCapacity(value);
      value = 0.;
      for (auto const& edgeId : pNode->outgoingEdges()) {
        auto* pStreet{&this->edge(edgeId)};
        value += pStreet->nLanes() * pStreet->transportCapacity();
      }
      pNode->setTransportCapacity(value == 0. ? 1. : value);
      if (pNode->capacity() == 0) {
        pNode->setCapacity(value);
      }
    }
  }

  void RoadNetwork::importTrafficLights(const std::string& fileName) {
    std::ifstream file{fileName};
    if (!file.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for reading.");
    }
    std::unordered_map<Id, Delay> storedGreenTimes;
    std::string line;
    std::getline(file, line);  // skip first line
    while (std::getline(file, line)) {
      if (line.empty()) {
        continue;
      }
      std::istringstream iss{line};
      std::string strId, streetSource, strCycleTime, strGT;
      // id;streetSource;cycleTime;greenTime
      std::getline(iss, strId, ';');
      std::getline(iss, streetSource, ';');
      std::getline(iss, strCycleTime, ';');
      std::getline(iss, strGT, '\n');

      auto const cycleTime{static_cast<Delay>(std::stoul(strCycleTime))};
      // Cast node(id) to traffic light
      auto const nodeId = static_cast<Id>(std::stoul(strId));
      auto& pNode = m_nodes.at(nodeId);
      if (!pNode->isTrafficLight()) {
        pNode = std::make_unique<TrafficLight>(
            pNode->id(), cycleTime, pNode->geometry().value());
      }
      auto& tl = static_cast<TrafficLight&>(*pNode);
      auto const streetId{edge(std::stoul(streetSource), pNode->id()).id()};
      auto const greenTime{static_cast<Delay>(std::stoul(strGT))};
      if (!storedGreenTimes.contains(pNode->id())) {
        storedGreenTimes.emplace(pNode->id(), greenTime);
      }
      auto const storedGT{storedGreenTimes.at(pNode->id())};
      if (storedGT == greenTime) {
        auto cycle = TrafficLightCycle(greenTime, 0);
        tl.setCycle(streetId, dsf::Direction::ANY, cycle);
      } else {
        auto cycle = TrafficLightCycle(greenTime, storedGT);
        tl.setCycle(streetId, dsf::Direction::ANY, cycle);
      }
    }
  }

  TrafficLight& RoadNetwork::makeTrafficLight(Id const nodeId,
                                              Delay const cycleTime,
                                              Delay const counter) {
    auto& pNode = m_nodes.at(nodeId);
    pNode = std::make_unique<TrafficLight>(*pNode, cycleTime, counter);
    return node<TrafficLight>(nodeId);
  }

  Roundabout& RoadNetwork::makeRoundabout(Id nodeId) {
    auto& pNode = m_nodes.at(nodeId);
    pNode = std::make_unique<Roundabout>(*pNode);
    return node<Roundabout>(nodeId);
  }

  Station& RoadNetwork::makeStation(Id nodeId, const unsigned int managementTime) {
    auto& pNode = m_nodes.at(nodeId);
    pNode = std::make_unique<Station>(*pNode, managementTime);
    return node<Station>(nodeId);
  }
  void RoadNetwork::addCoil(Id streetId, std::string const& name) {
    edge(streetId).enableCounter(name);
  }

  void RoadNetwork::addStreet(Street&& street) {
    m_capacity += street.capacity();
    auto const& geometry{street.geometry()};
    auto const& nodes{this->nodes()};
    if (!nodes.contains(street.source())) {
      spdlog::debug("Node with id {} not found, adding default", street.source());
      if (!geometry.empty()) {
        addNode<Intersection>(street.source(), geometry.front());
      } else {
        addNode<Intersection>(street.source());
      }
    }
    if (!nodes.contains(street.target())) {
      spdlog::debug("Node with id {} not found, adding default", street.target());
      if (!geometry.empty()) {
        addNode<Intersection>(street.target(), geometry.back());
      } else {
        addNode<Intersection>(street.target());
      }
    }
    addEdge<Street>(std::move(street));
  }

  void RoadNetwork::setStreetStatusById(Id const streetId, RoadStatus const status) {
    try {
      auto* pStreet{&edge(streetId)};
      pStreet->setStatus(status);
      spdlog::info("Changed status of {} to {}", *pStreet, status);
    } catch (const std::out_of_range&) {
      throw std::out_of_range(std::format("Street with id {} not found", streetId));
    }
  }
  void RoadNetwork::setStreetStatusByName(std::string const& streetName,
                                          RoadStatus const status) {
    std::atomic<std::size_t> nAffectedRoads{0};
    std::for_each(DSF_EXECUTION m_edges.cbegin(),
                  m_edges.cend(),
                  [this, &streetName, &status, &nAffectedRoads](auto const& pair) {
                    auto const& pStreet = pair.second;
                    if (pStreet->name().find(streetName) != std::string::npos) {
                      pStreet->setStatus(status);
                      ++nAffectedRoads;
                    }
                  });
    spdlog::info("Set status {} to {} streets with name containing \"{}\"",
                 status,
                 nAffectedRoads.load(),
                 streetName);
  }
  void RoadNetwork::changeStreetNLanesById(Id const streetId,
                                           int const nLanes,
                                           std::optional<double> const speedFactor) {
    try {
      edge(streetId).changeNLanes(nLanes, speedFactor);
    } catch (const std::out_of_range&) {
      throw std::out_of_range(std::format("Street with id {} not found", streetId));
    }
  }
  void RoadNetwork::changeStreetNLanesByName(std::string const& streetName,
                                             int const nLanes,
                                             std::optional<double> const speedFactor) {
    std::atomic<std::size_t> nAffectedRoads{0};
    std::for_each(
        DSF_EXECUTION m_edges.cbegin(),
        m_edges.cend(),
        [this, &streetName, &nLanes, &speedFactor, &nAffectedRoads](auto const& pair) {
          auto const& pStreet = pair.second;
          if (pStreet->name().find(streetName) != std::string::npos) {
            pStreet->changeNLanes(nLanes, speedFactor);
            ++nAffectedRoads;
          }
        });
    spdlog::info(
        "Changed number of lanes to {} for {} streets with name containing "
        "\"{}\"",
        nLanes,
        nAffectedRoads.load(),
        streetName);
  }
  void RoadNetwork::changeStreetCapacityById(Id const streetId, double const factor) {
    try {
      auto* pStreet{&edge(streetId)};
      auto const& currentCapacity{pStreet->capacity()};
      pStreet->setCapacity(std::ceil(currentCapacity * factor));
    } catch (const std::out_of_range&) {
      throw std::out_of_range(std::format("Street with id {} not found", streetId));
    }
  }
  void RoadNetwork::changeStreetCapacityByName(std::string const& streetName,
                                               double const factor) {
    std::atomic<std::size_t> nAffectedRoads{0};
    std::for_each(DSF_EXECUTION m_edges.cbegin(),
                  m_edges.cend(),
                  [this, &streetName, &factor, &nAffectedRoads](auto const& pair) {
                    auto const& pStreet = pair.second;
                    if (pStreet->name().find(streetName) != std::string::npos) {
                      auto const& currentCapacity = pStreet->capacity();
                      pStreet->setCapacity(std::ceil(currentCapacity * factor));
                      ++nAffectedRoads;
                    }
                  });
    spdlog::info(
        "Changed capacity by factor {} to {} streets with name containing \"{}\"",
        factor,
        nAffectedRoads.load(),
        streetName);
  }

  void RoadNetwork::setStreetStationaryWeights(
      std::unordered_map<Id, double> const& weights) {
    std::for_each(DSF_EXECUTION m_edges.cbegin(),
                  m_edges.cend(),
                  [this, &weights](auto const& pair) {
                    auto const streetId = pair.first;
                    auto const& pStreet = pair.second;
                    auto it = weights.find(streetId);
                    if (it != weights.end()) {
                      pStreet->setStationaryWeight(it->second);
                    } else {
                      pStreet->setStationaryWeight(1.0);
                    }
                  });
  }

  Street const* RoadNetwork::street(Id source, Id destination) const {
    auto const it = std::find_if(
        m_edges.cbegin(), m_edges.cend(), [source, destination](auto const& pair) {
          return pair.second->source() == source && pair.second->target() == destination;
        });
    if (it == m_edges.cend()) {
      return nullptr;
    }
    return it->second.get();
  }
}  // namespace dsf::mobility
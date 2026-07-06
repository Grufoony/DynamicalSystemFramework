#include "RoadNetwork.hpp"
#include "../geometry/Point.hpp"
#include "../geometry/PolyLine.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <ranges>

#include <csv.hpp>
#include <simdjson.h>
#include <tbb/parallel_for_each.h>

static constexpr auto EDGE_DEFAULT_ATTRIBUTES =
    std::to_array<std::string_view>({"id",
                                     "source",
                                     "target",
                                     "length",
                                     "maxspeed",
                                     "nlanes",
                                     "name",
                                     "type",
                                     "capacity",
                                     "status",
                                     "coilcode",
                                     "priority",
                                     "mobility_class",
                                     "forbidden_turns",
                                     "lane_mapping",
                                     "geometry"});

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
    // Create a vector of attributes not in EDGE_DEFAULT_ATTRIBUTES
    std::vector<std::string_view> additionalAttributes;
    for (auto const& colName : colNames) {
      if (std::find(EDGE_DEFAULT_ATTRIBUTES.begin(),
                    EDGE_DEFAULT_ATTRIBUTES.end(),
                    colName) == EDGE_DEFAULT_ATTRIBUTES.end()) {
        additionalAttributes.push_back(colName);
      }
    }
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
    bool const bHasPriority =
        (std::find(colNames.begin(), colNames.end(), "priority") != colNames.end());
    bool const bHasCapacity =
        (std::find(colNames.begin(), colNames.end(), "capacity") != colNames.end());
    bool const bHasStatus =
        (std::find(colNames.begin(), colNames.end(), "status") != colNames.end());
    bool const bHasForbiddenTurns =
        (std::find(colNames.begin(), colNames.end(), "forbidden_turns") !=
         colNames.end());
    bool const bHasMobilityClass =
        (std::find(colNames.begin(), colNames.end(), "mobility_class") != colNames.end());
    bool const bHasLaneMapping =
        (std::find(colNames.begin(), colNames.end(), "lane_mapping") != colNames.end());

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

      if (!strType.empty() && !bHasMobilityClass) {
        std::transform(
            strType.begin(), strType.end(), strType.begin(), [](unsigned char c) {
              return std::tolower(c);
            });
        if (strType.find("motorway") != std::string::npos) {
          edge(streetId).setMobilityClass(128u);
        } else if (strType.find("primary") != std::string::npos) {
          edge(streetId).setMobilityClass(64u);
        } else if (strType.find("secondary") != std::string::npos) {
          edge(streetId).setMobilityClass(32u);
        } else if (strType.find("tertiary") != std::string::npos) {
          edge(streetId).setMobilityClass(16u);
        } else if (strType.find("residential") != std::string::npos) {
          edge(streetId).setMobilityClass(0u);
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

      // Parse capacity field if present
      if (bHasCapacity) {
        try {
          int capacityValue = row["capacity"].get<int>();
          edge(streetId).setCapacity(capacityValue);
        } catch (...) {
          spdlog::warn("Invalid capacity for edge {}. Using default.", streetId);
        }
      }

      // Parse status field if present
      if (bHasStatus) {
        try {
          auto statusStr = row["status"].get<std::string>();
          std::transform(
              statusStr.begin(), statusStr.end(), statusStr.begin(), [](unsigned char c) {
                return std::tolower(c);
              });
          if (statusStr == "closed") {
            edge(streetId).setStatus(RoadStatus::CLOSED);
          } else if (statusStr == "open" || statusStr.empty()) {
            edge(streetId).setStatus(RoadStatus::OPEN);
          } else {
            spdlog::warn(
                "Unknown status '{}' for edge {}. Valid values: 'open', 'closed'. Using "
                "'open'.",
                statusStr,
                streetId);
            edge(streetId).setStatus(RoadStatus::OPEN);
          }
        } catch (...) {
          spdlog::warn("Invalid status for edge {}. Using default (OPEN).", streetId);
          edge(streetId).setStatus(RoadStatus::OPEN);
        }
      }
      // Handle mobility_class field if present
      if (bHasMobilityClass) {
        try {
          auto mobilityClassValue = row["mobility_class"].get<std::uint8_t>();
          edge(streetId).setMobilityClass(mobilityClassValue);
        } catch (...) {
          spdlog::warn("Invalid mobility_class for edge {}. Using default (0).",
                       streetId);
          edge(streetId).setMobilityClass(0u);
        }
      }
      // Parse forbidden_turns field if present
      if (bHasForbiddenTurns) {
        // Expect a string of the form [edgeId1, edgeId2, ...]
        try {
          auto forbiddenTurnsStr = row["forbidden_turns"].get<std::string>();
          std::replace(forbiddenTurnsStr.begin(), forbiddenTurnsStr.end(), '[', ' ');
          std::replace(forbiddenTurnsStr.begin(), forbiddenTurnsStr.end(), ']', ' ');
          std::replace(forbiddenTurnsStr.begin(), forbiddenTurnsStr.end(), ',', ' ');

          std::set<Id> forbiddenTurnsSet;

          std::stringstream ss(forbiddenTurnsStr);
          std::string turn;
          while (ss >> turn) {
            forbiddenTurnsSet.insert(static_cast<Id>(std::stoull(turn)));
          }
          if (!forbiddenTurnsSet.empty()) {
            edge(streetId).setForbiddenTurns(forbiddenTurnsSet);
          }
        } catch (...) {
          spdlog::error("Invalid forbidden_turns for edge {} ({}).",
                        streetId,
                        row["forbidden_turns"].get<std::string>());
        }
      }
      if (bHasLaneMapping) {
        // Expect a string of the form [straight, left, right, ...]
        try {
          auto laneMappingStr = row["lane_mapping"].get<std::string>();
          std::replace(laneMappingStr.begin(), laneMappingStr.end(), '[', ' ');
          std::replace(laneMappingStr.begin(), laneMappingStr.end(), ']', ' ');
          std::replace(laneMappingStr.begin(), laneMappingStr.end(), ',', ' ');

          std::vector<Direction> laneMappingVec;
          std::stringstream ss(laneMappingStr);
          std::string lane;
          while (ss >> lane) {
            std::transform(lane.begin(), lane.end(), lane.begin(), [](unsigned char c) {
              return std::tolower(c);
            });
            // Check if "straight" is substring of lane
            if (lane.find("straight") != std::string::npos) {
              if (lane.find("left") != std::string::npos) {
                laneMappingVec.push_back(Direction::LEFTANDSTRAIGHT);
              } else if (lane.find("right") != std::string::npos) {
                laneMappingVec.push_back(Direction::RIGHTANDSTRAIGHT);
              } else {
                laneMappingVec.push_back(Direction::STRAIGHT);
              }
            } else if (lane.find("left") != std::string::npos) {
              laneMappingVec.push_back(Direction::LEFT);
            } else if (lane.find("right") != std::string::npos) {
              laneMappingVec.push_back(Direction::RIGHT);
            } else {
              laneMappingVec.push_back(Direction::ANY);
            }
          }
          if (!laneMappingVec.empty()) {
            std::sort(laneMappingVec.begin(), laneMappingVec.end());
            edge(streetId).setLaneMapping(laneMappingVec);
          }
        } catch (...) {
          spdlog::error("Invalid lane_mapping for edge {} ({}).",
                        streetId,
                        row["lane_mapping"].get<std::string>());
        }
      }

      // Handle additional attributes
      for (auto const& attrName : additionalAttributes) {
        auto const strAttrName = std::string(attrName);
        auto const attrValue = row[strAttrName].get<std::string>();
        auto& e = edge(streetId);

        if (attrValue.empty()) {
          e.setAttribute(strAttrName, std::monostate{});
        } else if (attrValue == "true" || attrValue == "false") {
          e.setAttribute(strAttrName, attrValue == "true");
        } else {
          std::size_t const valueSize = attrValue.size();
          std::size_t pos{0};

          // Try casting to int
          try {
            auto const value = std::stoll(attrValue, &pos);
            if (pos == valueSize) {
              e.setAttribute(strAttrName, value);
              continue;
            }
          } catch (std::exception const&) {
            // Not an int, try double
          }
          try {
            auto const value = std::stod(attrValue, &pos);
            if (pos == valueSize) {
              e.setAttribute(strAttrName, value);
              continue;
            }
          } catch (std::exception const&) {
            // Not a double, keep as string
          }
          e.setAttribute(strAttrName, attrValue);  // Fall back to string
        }
      }
    }
    this->m_nodes.rehash(0);
    this->m_edges.rehash(0);
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
        makeTrafficLight(nodeId);
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
      auto const mobilityClassResult = edge_properties.at_key("mobility_class");
      if (!mobilityClassResult.error()) {
        if (mobilityClassResult.is_uint64()) {
          edge(edge_id).setMobilityClass(
              static_cast<std::uint8_t>(mobilityClassResult.get_uint64()));
        } else if (mobilityClassResult.is_int64()) {
          edge(edge_id).setMobilityClass(
              static_cast<std::uint8_t>(mobilityClassResult.get_int64()));
        } else {
          spdlog::warn("Invalid mobility_class for edge {}, adding default (0u)",
                       edge_id);
          edge(edge_id).setMobilityClass(0u);
        }
      } else if (!strType.empty()) {
        std::transform(
            strType.begin(), strType.end(), strType.begin(), [](unsigned char c) {
              return std::tolower(c);
            });
        if (strType.find("motorway") != std::string::npos) {
          edge(edge_id).setMobilityClass(128u);
        } else if (strType.find("primary") != std::string::npos) {
          edge(edge_id).setMobilityClass(64u);
        } else if (strType.find("secondary") != std::string::npos) {
          edge(edge_id).setMobilityClass(32u);
        } else if (strType.find("tertiary") != std::string::npos) {
          edge(edge_id).setMobilityClass(16u);
        } else if (strType.find("residential") != std::string::npos) {
          edge(edge_id).setMobilityClass(0u);
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
      // Check if there is a forbidden_turns property
      auto const forbidden_turns_result = edge_properties.at_key("forbidden_turns");
      if (!forbidden_turns_result.error() && !forbidden_turns_result.is_null()) {
        if (forbidden_turns_result.is_array()) {
          auto const turnsArray = forbidden_turns_result.get_array().value();
          if (turnsArray.size() > 0) {
            std::set<Id> forbiddenTurnsSet;
            for (auto const& turn : turnsArray) {
              if (turn.is_uint64()) {
                forbiddenTurnsSet.insert(static_cast<Id>(turn.get_uint64()));
              } else if (turn.is_int64()) {
                forbiddenTurnsSet.insert(static_cast<Id>(turn.get_int64()));
              } else {
                spdlog::warn(
                    "Invalid forbidden turn value for edge {}, skipping this turn",
                    edge_id);
              }
            }
            if (!forbiddenTurnsSet.empty()) {
              edge(edge_id).setForbiddenTurns(forbiddenTurnsSet);
            }
          }
        } else {
          spdlog::warn(
              "Invalid forbidden_turns property for edge {}, expected an array, skipping",
              edge_id);
        }
      }
      // Handle lane_mapping property
      auto const lane_mapping_result = edge_properties.at_key("lane_mapping");
      if (!lane_mapping_result.error() && !lane_mapping_result.is_null()) {
        if (lane_mapping_result.is_array()) {
          std::vector<Direction> laneMappingVec;
          auto const laneMappingArray = lane_mapping_result.get_array().value();
          for (auto const lane : laneMappingArray) {
            if (lane.is_string()) {
              std::string laneStr{lane.get_string().value()};
              std::transform(
                  laneStr.begin(), laneStr.end(), laneStr.begin(), [](unsigned char c) {
                    return std::tolower(c);
                  });
              if (laneStr.find("straight") != std::string::npos) {
                if (laneStr.find("left") != std::string::npos) {
                  laneMappingVec.push_back(Direction::LEFTANDSTRAIGHT);
                } else if (laneStr.find("right") != std::string::npos) {
                  laneMappingVec.push_back(Direction::RIGHTANDSTRAIGHT);
                } else {
                  laneMappingVec.push_back(Direction::STRAIGHT);
                }
              } else if (laneStr.find("left") != std::string::npos) {
                laneMappingVec.push_back(Direction::LEFT);
              } else if (laneStr.find("right") != std::string::npos) {
                laneMappingVec.push_back(Direction::RIGHT);
              } else {
                laneMappingVec.push_back(Direction::ANY);
              }
            } else {
              spdlog::warn(
                  "Invalid lane mapping value for edge {}, expected a string, skipping "
                  "this lane",
                  edge_id);
            }
          }
          if (!laneMappingVec.empty()) {
            std::sort(laneMappingVec.begin(), laneMappingVec.end());
            edge(edge_id).setLaneMapping(laneMappingVec);
          }
        } else {
          spdlog::warn(
              "Invalid lane_mapping property for edge {}, expected an array, skipping",
              edge_id);
        }
      }
      // Handle additional attributes
      for (auto const& [attrName, attrValue] : edge_properties.get_object()) {
        if (std::find(EDGE_DEFAULT_ATTRIBUTES.begin(),
                      EDGE_DEFAULT_ATTRIBUTES.end(),
                      attrName) != EDGE_DEFAULT_ATTRIBUTES.end()) {
          continue;  // Skip default attributes
        }
        auto const strAttrName{std::string(attrName)};
        if (attrValue.is_null()) {
          edge(edge_id).setAttribute(strAttrName, std::monostate{});
        } else if (attrValue.is_bool()) {
          edge(edge_id).setAttribute(strAttrName, attrValue.get_bool());
        } else if (attrValue.is_number()) {
          if (attrValue.is_int64()) {
            edge(edge_id).setAttribute(strAttrName, attrValue.get_int64());
          } else {
            edge(edge_id).setAttribute(strAttrName, attrValue.get_double());
          }
        } else if (attrValue.is_string()) {
          edge(edge_id).setAttribute(strAttrName,
                                     std::string(attrValue.get_string().value()));
        } else {
          spdlog::warn("Unsupported attribute type for attribute {} of edge {}",
                       strAttrName,
                       edge_id);
        }
      }
    }
    this->m_nodes.rehash(0);
    this->m_edges.rehash(0);
  }

  RoadNetwork::RoadNetwork() { this->setEdgeWeight("traveltime"); }

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

  void RoadNetwork::autoInitTrafficLights(const double mainRoadPercentage,
                                          const dsf::Delay defaultCycleDuration) {
    tbb::parallel_for_each(
        m_nodes.begin(),
        m_nodes.end(),
        [this, mainRoadPercentage, defaultCycleDuration](auto& pair) {
          auto& pNode = pair.second;
          if (!pNode->isTrafficLight()) {
            return;
          }
          auto& tl = static_cast<TrafficLight&>(*pNode);
          if (!tl.streetPriorities().empty() || !tl.phases().empty()) {
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

          // Build two phases: priority streets (phase 0) and non-priority (phase 1).
          auto const mainGreenTime{
              static_cast<Delay>(mainRoadPercentage * defaultCycleDuration)};
          auto const secondaryGreenTime{
              static_cast<Delay>(defaultCycleDuration - mainGreenTime)};

          TrafficLightPhase priorityPhase{mainGreenTime};
          TrafficLightPhase nonPriorityPhase{secondaryGreenTime};

          std::for_each(
              inNeighbours.begin(), inNeighbours.end(), [&](auto const& edgeId) {
                auto const& rStreet = this->edge(edgeId);
                auto const streetId{rStreet.id()};
                auto const nLane{nLanes.at(streetId)};
                bool const isPriority{tl.streetPriorities().contains(streetId)};
                auto& targetPhase{isPriority ? priorityPhase : nonPriorityPhase};

                spdlog::debug("Adding street {} to {} phase (nLanes={})",
                              streetId,
                              isPriority ? "priority" : "non-priority",
                              nLane);

                targetPhase.addGreen(streetId);  // Direction::ANY
              });

          tl.setPhases({priorityPhase, nonPriorityPhase});
        });
  }
  void RoadNetwork::autoMapStreetLanes() {
    spdlog::warn(
        "NOTE: this function is still experimental and may not work correctly on real "
        "road networks. Use with caution.");
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
                // Collect all directions configured for this street across all phases.
                std::unordered_set<Direction> tlDirs;
                for (auto const& phase : tl.phases()) {
                  auto const streetIt = phase.greenSet().find(pInStreet->id());
                  if (streetIt == phase.greenSet().end())
                    continue;
                  for (auto const dir : streetIt->second)
                    tlDirs.insert(dir);
                }
                if (!tlDirs.empty()) {
                  if (tlDirs.size() == static_cast<std::size_t>(nLanes)) {
                    // Phase directions match lane count — use them directly.
                    spdlog::debug(
                        "Using traffic light {} phase directions for street {} -> {}",
                        tl.id(),
                        pInStreet->source(),
                        pInStreet->target());
                    allowedTurns.clear();
                    for (auto const dir : tlDirs)
                      allowedTurns.emplace(dir);
                  } else if (tlDirs.contains(Direction::LEFTANDSTRAIGHT)) {
                    allowedTurns.erase(Direction::LEFT);
                    allowedTurns.erase(Direction::STRAIGHT);
                    allowedTurns.emplace(Direction::LEFTANDSTRAIGHT);
                  } else if (tlDirs.contains(Direction::RIGHTANDSTRAIGHT)) {
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
                    // Check if any phase configures LEFTANDSTRAIGHT+RIGHT for this street.
                    bool hasLeftAndStraight{false}, hasRight{false};
                    for (auto const& phase : tl.phases()) {
                      auto const streetIt = phase.greenSet().find(pInStreet->id());
                      if (streetIt == phase.greenSet().end())
                        continue;
                      hasLeftAndStraight |=
                          streetIt->second.contains(Direction::LEFTANDSTRAIGHT);
                      hasRight |= streetIt->second.contains(Direction::RIGHT);
                    }
                    if (hasLeftAndStraight && hasRight) {
                      allowedTurns.erase(Direction::LEFT);
                      allowedTurns.erase(Direction::STRAIGHT);
                      allowedTurns.emplace(Direction::LEFTANDSTRAIGHT);
                      break;
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
    std::atomic<std::size_t> nAssigned{0}, nNotAssigned{0};
    tbb::parallel_for_each(
        m_nodes.cbegin(),
        m_nodes.cend(),
        [this, &nAssigned, &nNotAssigned](auto const& pair) {
          auto const& pNode{pair.second};
          auto const& inNeighbours{pNode->ingoingEdges()};
          // NOTE: std::multimap iterates keys in descending order of std::uint8_t.
          // std::uint8_t is defined so that more important roads (e.g., HIGHWAY = 128u,
          // PRIMARY = 64u, SECONDARY = 32u, ...) have bigger values. The logic
          // below relies on this ordering to consider higher-priority road types
          // first when selecting streets to mark as priority roads.
          std::multimap<std::uint8_t, Id, std::greater<std::uint8_t>> types;
          for (auto const& edgeId : inNeighbours) {
            auto* pStreet{&this->edge(edgeId)};
            auto const mobilityClass = pStreet->mobilityClass();
            if (mobilityClass != 0u) {
              types.emplace(mobilityClass, pStreet->id());
            }
          }
          if (types.size() < 2) {
            ++nNotAssigned;
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
            spdlog::debug("{}: unable to auto-assign road priorities", *pNode);
            ++nNotAssigned;
            return;
          }

          for (auto const& streetId : priorityRoads) {
            auto* pStreet{&this->edge(streetId)};
            pStreet->setPriority();
            spdlog::debug("Setting priority to street {}", pStreet->id());
            ++nAssigned;
          }
        });
    spdlog::info(
        "Correctly assigned road priorities to {} streets, failed to assign to {} "
        "streets.",
        nAssigned.load(),
        nNotAssigned.load());
  }

  void RoadNetwork::setEdgeWeight(std::string_view const strv_weight,
                                  std::optional<double> const threshold) {
    if (strv_weight == "traveltime") {
      m_weightFunction = [](Street const& street) {
        return street.estimatedTravelTime();
      };
    } else if (strv_weight == "length") {
      m_weightFunction = [](Street const& street) { return street.length(); };
    } else if (strv_weight == "uniform") {
      m_weightFunction = []([[maybe_unused]] Street const& street) { return 1.0; };
    } else {  // Custom attribute
      m_weightFunction = [strv_weight](Street const& street) {
        auto it = std::find_if(
            street.attributes().cbegin(),
            street.attributes().cend(),
            [strv_weight](auto const& pair) { return pair.first == strv_weight; });
        if (it == street.attributes().end()) {
          throw std::runtime_error(std::format(
              "Attribute {} not found in street {}", strv_weight, street.id()));
        }
        auto const& attrValue = it->second;
        return std::visit(
            [](auto&& value) -> double {
              using T = std::decay_t<decltype(value)>;
              if constexpr (std::is_arithmetic_v<T>) {
                return static_cast<double>(value);
              } else {
                throw std::runtime_error(
                    "Custom weight attribute is not a numeric type.");
              }
            },
            attrValue);
      };
    }
    m_weightThreshold = threshold;
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

    // ── Parse all rows into per-node entry lists ──────────────────────────
    struct TLEntry {
      Id streetId;
      Delay cycleTime;
      Delay greenTime;
    };
    std::unordered_map<Id, std::vector<TLEntry>> entriesPerNode;

    std::string line;
    std::getline(file, line);  // skip header
    while (std::getline(file, line)) {
      if (line.empty())
        continue;
      std::istringstream iss{line};
      std::string strId, strSource, strCycle, strGreen;
      // Format: id;sourceNodeId;cycleTime;greenTime
      std::getline(iss, strId, ';');
      std::getline(iss, strSource, ';');
      std::getline(iss, strCycle, ';');
      std::getline(iss, strGreen, '\n');

      auto const nodeId = static_cast<Id>(std::stoul(strId));
      auto const cycleUl = std::stoul(strCycle);
      auto const greenUl = std::stoul(strGreen);
      if (cycleUl > std::numeric_limits<Delay>::max() ||
          greenUl > std::numeric_limits<Delay>::max()) {
        throw std::invalid_argument(std::format(
            "importTrafficLights: cycleTime/greenTime out of range for node {}.",
            nodeId));
      }
      auto const cycleTime = static_cast<Delay>(cycleUl);
      auto const greenTime = static_cast<Delay>(greenUl);
      if (greenTime > cycleTime) {
        throw std::invalid_argument(std::format(
            "importTrafficLights: greenTime ({}) exceeds cycleTime ({}) for node {}.",
            greenTime,
            cycleTime,
            nodeId));
      }
      auto const streetId = edge(std::stoul(strSource), nodeId).id();

      auto& list = entriesPerNode[nodeId];
      if (!list.empty() && list.front().cycleTime != cycleTime) {
        throw std::invalid_argument(std::format(
            "importTrafficLights: inconsistent cycleTime for node {} ({} vs {}).",
            nodeId,
            cycleTime,
            list.front().cycleTime));
      }
      list.push_back({streetId, cycleTime, greenTime});
    }

    // ── Convert each node's entries into a two-phase TrafficLight ─────────
    for (auto& [nodeId, entries] : entriesPerNode) {
      auto& pNode = m_nodes.at(nodeId);

      // Promote the node to TrafficLight if it isn't one already.
      if (!pNode->isTrafficLight()) {
        pNode = std::make_unique<TrafficLight>(*pNode);
      }
      auto& tl = static_cast<TrafficLight&>(*pNode);

      // The first greenTime seen for this node is the "phase 0" duration.
      // Streets with the same greenTime → Phase 0.
      // Streets with a different greenTime → Phase 1.
      auto const firstGreenTime = entries.front().greenTime;
      auto const cycleTime = entries.front().cycleTime;
      auto const secondaryGreenTime = static_cast<Delay>(cycleTime - firstGreenTime);

      TrafficLightPhase phase0{firstGreenTime};
      TrafficLightPhase phase1{secondaryGreenTime};

      for (auto const& entry : entries) {
        if (entry.greenTime == firstGreenTime) {
          phase0.addGreen(entry.streetId);  // Direction::ANY
        } else {
          phase1.addGreen(entry.streetId);  // Direction::ANY
        }
      }

      tl.setPhases({phase0, phase1});
      spdlog::debug(
          "importTrafficLights: node {} → phase0 ({} ticks, {} streets) + "
          "phase1 ({} ticks, {} streets).",
          nodeId,
          firstGreenTime,
          phase0.greenSet().size(),
          secondaryGreenTime,
          phase1.greenSet().size());
    }
  }

  TrafficLight& RoadNetwork::makeTrafficLight(Id const nodeId) {
    auto& pNode = m_nodes.at(nodeId);
    pNode = std::make_unique<TrafficLight>(*pNode);
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
                  [&streetName, &status, &nAffectedRoads](auto const& pair) {
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
        [&streetName, &nLanes, &speedFactor, &nAffectedRoads](auto const& pair) {
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
                  [&streetName, &factor, &nAffectedRoads](auto const& pair) {
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
  void RoadNetwork::exportCSV(std::string_view const folder) const {
    auto const nodeTypeToString = [](RoadJunction const& junction) {
      if (junction.isTrafficLight()) {
        return std::string{"traffic_signals"};
      }
      if (junction.isRoundabout()) {
        return std::string{"roundabout"};
      }
      return std::string{"intersection"};
    };

    auto const attributeValueToString =
        [](std::variant<std::monostate, bool, std::int64_t, double, std::string> const&
               value) {
          return std::visit(
              [](auto const& typedValue) -> std::string {
                using T = std::decay_t<decltype(typedValue)>;
                if constexpr (std::is_same_v<T, std::monostate>) {
                  return std::string{};
                } else {
                  return std::format("{}", typedValue);
                }
              },
              value);
        };

    std::vector<std::string> extraEdgeColumns;
    for (auto const& [_, pStreet] : m_edges) {
      for (auto const& [attrName, _] : pStreet->attributes()) {
        if (std::find(EDGE_DEFAULT_ATTRIBUTES.begin(),
                      EDGE_DEFAULT_ATTRIBUTES.end(),
                      attrName) != EDGE_DEFAULT_ATTRIBUTES.end()) {
          continue;
        }
        if (std::find(extraEdgeColumns.begin(), extraEdgeColumns.end(), attrName) ==
            extraEdgeColumns.end()) {
          extraEdgeColumns.push_back(attrName);
        }
      }
    }
    std::ranges::sort(extraEdgeColumns);

    // Check if path is a directory
    if (!std::filesystem::is_directory(folder)) {
      throw std::runtime_error(std::format("Path {} is not a directory", folder));
    }
    auto const edgesPath = std::filesystem::path(folder) / "edges.csv";
    auto const nodesPath = std::filesystem::path(folder) / "nodes.csv";

    // Save edges
    {
      std::ofstream edgesFile{edgesPath};
      if (!edgesFile.is_open()) {
        throw std::runtime_error(
            std::format("Error opening file {} for writing", edgesPath.string()));
      }
      csv::CSVWriter writer{edgesFile};

      std::vector<std::string> edgesHeader{"id",
                                           "source",
                                           "target",
                                           "length",
                                           "maxspeed",
                                           "nlanes",
                                           "type",
                                           "capacity",
                                           "status",
                                           "name",
                                           "priority",
                                           "coilcode",
                                           "geometry"};
      edgesHeader.insert(
          edgesHeader.end(), extraEdgeColumns.begin(), extraEdgeColumns.end());
      writer << edgesHeader;

      for (auto const& [_, pStreet] : m_edges) {
        auto const strGeometry = pStreet->geometry().empty()
                                     ? std::string{}
                                     : std::format("{}", pStreet->geometry());
        std::vector<std::string> edgeRow;
        edgeRow.reserve(edgesHeader.size());
        edgeRow.emplace_back(std::format("{}", pStreet->id()));
        edgeRow.emplace_back(std::format("{}", pStreet->source()));
        edgeRow.emplace_back(std::format("{}", pStreet->target()));
        edgeRow.emplace_back(std::format("{}", pStreet->length()));
        edgeRow.emplace_back(std::format("{}", pStreet->maxSpeed() * 3.6));
        edgeRow.emplace_back(std::format("{}", pStreet->nLanes()));
        edgeRow.emplace_back(std::format("{}", pStreet->mobilityClass()));
        edgeRow.emplace_back(std::format("{}", pStreet->capacity()));
        edgeRow.emplace_back(std::format("{}", pStreet->roadStatus()));
        edgeRow.emplace_back(pStreet->name());
        edgeRow.emplace_back(std::format("{}", pStreet->hasPriority()));
        edgeRow.emplace_back(pStreet->hasCoil() ? pStreet->counterName() : std::string{});
        edgeRow.emplace_back(strGeometry);
        for (auto const& attrName : extraEdgeColumns) {
          auto const it = pStreet->attributes().find(attrName);
          if (it == pStreet->attributes().end()) {
            edgeRow.emplace_back();
            continue;
          }
          edgeRow.emplace_back(attributeValueToString(it->second));
        }
        writer << edgeRow;
      }
    }

    // Save nodes
    {
      std::ofstream nodesFile{nodesPath};
      if (!nodesFile.is_open()) {
        throw std::runtime_error(
            std::format("Error opening file {} for writing", nodesPath.string()));
      }
      csv::CSVWriter writer{nodesFile};
      writer << std::array<std::string, 6>{
          "id", "type", "geometry", "capacity", "transportCapacity", "name"};
      for (auto const& [_, pNode] : m_nodes) {
        auto const strGeometry = pNode->geometry().has_value()
                                     ? std::format("{}", pNode->geometry().value())
                                     : std::string{};
        writer << std::array<std::string, 6>{
            std::format("{}", pNode->id()),
            nodeTypeToString(*pNode),
            strGeometry,
            std::format("{}", pNode->capacity()),
            std::format("{}", pNode->transportCapacity()),
            pNode->name()};
      }
    }
  }
}  // namespace dsf::mobility
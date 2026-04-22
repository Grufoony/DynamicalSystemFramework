#include "dsf.hpp"

#include ".docstrings.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>         // Changed to include all stl type casters
#include <pybind11/functional.h>  // For std::function support
#include <pybind11/numpy.h>       // For numpy array support

#include <spdlog/spdlog.h>  // For logging functionality

PYBIND11_MODULE(dsf_cpp, m) {
  m.doc() = "Python bindings for the DSF library";
  m.attr("__version__") = dsf::version();

  // Bind the dsf::log_to_file function
  m.def("log_to_file",
        &dsf::log_to_file,
        pybind11::arg("path"),
        "Set up logging to a specified file");

  // Create mobility submodule
  auto mobility = m.def_submodule("mobility",
                                  "Bindings for mobility-related classes and functions, "
                                  "under the dsf::mobility C++ namespace.");
  auto mdt = m.def_submodule("mdt",
                             "Bindings for movement data tools (MDT) related classes and "
                             "functions, under the dsf::mdt C++ namespace.");

  // Bind AgentInsertionMethod enum
  pybind11::enum_<dsf::mobility::AgentInsertionMethod>(mobility, "AgentInsertionMethod")
      .value("ODS", dsf::mobility::AgentInsertionMethod::ODS)
      .value("RANDOM", dsf::mobility::AgentInsertionMethod::RANDOM)
      .value("RANDOM_ODS", dsf::mobility::AgentInsertionMethod::RANDOM_ODS)
      .export_values();

  // Bind PathWeight enum
  pybind11::enum_<dsf::PathWeight>(mobility, "PathWeight")
      .value("LENGTH", dsf::PathWeight::LENGTH)
      .value("TRAVELTIME", dsf::PathWeight::TRAVELTIME)
      .value("WEIGHT", dsf::PathWeight::WEIGHT)
      .export_values();

  // Bind TrafficLightOptimization enum
  pybind11::enum_<dsf::TrafficLightOptimization>(mobility, "TrafficLightOptimization")
      .value("SINGLE_TAIL", dsf::TrafficLightOptimization::SINGLE_TAIL)
      .value("DOUBLE_TAIL", dsf::TrafficLightOptimization::DOUBLE_TAIL)
      .export_values();

  // Bind SpeedFunction enum
  pybind11::enum_<dsf::SpeedFunction>(mobility, "SpeedFunction")
      .value("CUSTOM", dsf::SpeedFunction::CUSTOM)
      .value("LINEAR", dsf::SpeedFunction::LINEAR)
      .export_values();

  // Bind RoadStatus enum
  pybind11::enum_<dsf::mobility::RoadStatus>(mobility, "RoadStatus")
      .value("OPEN", dsf::mobility::RoadStatus::OPEN)
      .value("CLOSED", dsf::mobility::RoadStatus::CLOSED)
      .export_values();

  // Bind spdlog log level enum
  pybind11::enum_<spdlog::level::level_enum>(m, "LogLevel")
      .value("TRACE", spdlog::level::trace)
      .value("DEBUG", spdlog::level::debug)
      .value("INFO", spdlog::level::info)
      .value("WARN", spdlog::level::warn)
      .value("ERROR", spdlog::level::err)
      .value("CRITICAL", spdlog::level::critical)
      .value("OFF", spdlog::level::off)
      .export_values();

  // Bind spdlog logging functions
  m.def("set_log_level",
        &spdlog::set_level,
        pybind11::arg("level"),
        "Set the global log level for spdlog");

  m.def("get_log_level", &spdlog::get_level, "Get the current global log level");

  // Bind Street class to mobility submodule
  pybind11::class_<dsf::mobility::Street>(mobility, "Street")
      .def(
          "id", &dsf::mobility::Street::id, dsf::g_docstrings.at("dsf::Edge::id").c_str())
      .def("source",
           &dsf::mobility::Street::source,
           dsf::g_docstrings.at("dsf::Edge::source").c_str())
      .def("target",
           &dsf::mobility::Street::target,
           dsf::g_docstrings.at("dsf::Edge::target").c_str())
      .def("geometry",
           &dsf::mobility::Street::geometry,
           dsf::g_docstrings.at("dsf::Edge::geometry").c_str())
      .def("name",
           &dsf::mobility::Street::name,
           dsf::g_docstrings.at("dsf::mobility::Road::name").c_str())
      .def("length",
           &dsf::mobility::Street::length,
           dsf::g_docstrings.at("dsf::mobility::Road::length").c_str())
      .def("maxSpeed",
           &dsf::mobility::Street::maxSpeed,
           dsf::g_docstrings.at("dsf::mobility::Road::maxSpeed").c_str());

  // Bind RoadJunction class to mobility submodule
  pybind11::class_<dsf::mobility::RoadJunction>(mobility, "RoadJunction")
      .def("id",
           &dsf::mobility::RoadJunction::id,
           dsf::g_docstrings.at("dsf::Node::id").c_str())
      .def("geometry",
           &dsf::mobility::RoadJunction::geometry,
           dsf::g_docstrings.at("dsf::Node::geometry").c_str())
      .def("capacity",
           &dsf::mobility::RoadJunction::capacity,
           dsf::g_docstrings.at("dsf::mobility::RoadJunction::capacity").c_str())
      .def(
          "transportCapacity",
          &dsf::mobility::RoadJunction::transportCapacity,
          dsf::g_docstrings.at("dsf::mobility::RoadJunction::transportCapacity").c_str());

  // Bind Measurement to main module (can be used across different contexts)
  pybind11::class_<dsf::Measurement<double>>(m, "Measurement")
      .def(pybind11::init<double, double, std::size_t>(),
           pybind11::arg("mean"),
           pybind11::arg("std"),
           pybind11::arg("n"),
           dsf::g_docstrings.at("dsf::Measurement::Measurement").c_str())
      .def_readwrite("mean",
                     &dsf::Measurement<double>::mean,
                     dsf::g_docstrings.at("dsf::Measurement::mean").c_str())
      .def_readwrite("std",
                     &dsf::Measurement<double>::std,
                     dsf::g_docstrings.at("dsf::Measurement::std").c_str())
      .def_readwrite("n",
                     &dsf::Measurement<double>::n,
                     dsf::g_docstrings.at("dsf::Measurement::n").c_str())
      .def_readwrite("is_valid",
                     &dsf::Measurement<double>::is_valid,
                     dsf::g_docstrings.at("dsf::Measurement::is_valid").c_str());

  // Bind mobility-related classes to mobility submodule
  pybind11::class_<dsf::mobility::RoadNetwork>(mobility, "RoadNetwork")
      .def(pybind11::init<>(),
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::RoadNetwork").c_str())
      .def("nNodes",
           &dsf::mobility::RoadNetwork::nNodes,
           dsf::g_docstrings.at("dsf::Network::nNodes").c_str())
      .def("nEdges",
           &dsf::mobility::RoadNetwork::nEdges,
           dsf::g_docstrings.at("dsf::Network::nEdges").c_str())
      .def("nCoils",
           &dsf::mobility::RoadNetwork::nCoils,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::nCoils").c_str())
      .def("nIntersections",
           &dsf::mobility::RoadNetwork::nIntersections,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::nIntersections").c_str())
      .def("nRoundabouts",
           &dsf::mobility::RoadNetwork::nRoundabouts,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::nRoundabouts").c_str())
      .def("nTrafficLights",
           &dsf::mobility::RoadNetwork::nTrafficLights,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::nTrafficLights").c_str())
      // Bind node and edge Network accessors, which return a ref or a cost ref
      // node should return a RoadJunction and edge should return a Street
      .def(
          "node",
          static_cast<dsf::mobility::RoadJunction& (
              dsf::mobility::RoadNetwork::*)(dsf::Id)>(&dsf::mobility::RoadNetwork::node),
          pybind11::arg("nodeId"),
          dsf::g_docstrings.at("dsf::Network::node").c_str())
      .def("edge",
           static_cast<dsf::mobility::Street& (dsf::mobility::RoadNetwork::*)(dsf::Id)>(
               &dsf::mobility::RoadNetwork::edge),
           pybind11::arg("edgeId"),
           dsf::g_docstrings.at("dsf::Network::edge").c_str())
      .def("capacity",
           &dsf::mobility::RoadNetwork::capacity,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::capacity").c_str())
      .def(
          "adjustNodeCapacities",
          &dsf::mobility::RoadNetwork::adjustNodeCapacities,
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::adjustNodeCapacities").c_str())
      .def("autoInitTrafficLights",
           &dsf::mobility::RoadNetwork::autoInitTrafficLights,
           pybind11::arg("mainRoadPercentage") = 0.6,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::autoInitTrafficLights")
               .c_str())
      .def("autoMapStreetLanes",
           &dsf::mobility::RoadNetwork::autoMapStreetLanes,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::autoMapStreetLanes").c_str())
      .def(
          "describe",
          [](dsf::mobility::RoadNetwork& self) {
            self.describe();  // Uses default std::cout
          },
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::describe").c_str())
      .def("autoAssignRoadPriorities",
           &dsf::mobility::RoadNetwork::autoAssignRoadPriorities,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::autoAssignRoadPriorities")
               .c_str())
      .def("setStreetStationaryWeights",
           &dsf::mobility::RoadNetwork::setStreetStationaryWeights,
           pybind11::arg("weights"),
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::setStreetStationaryWeights")
               .c_str())
      .def(
          "importEdges",
          [](dsf::mobility::RoadNetwork& self, const std::string& fileName) {
            self.importEdges(fileName);
          },
          pybind11::arg("fileName"),
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::importEdges").c_str())
      .def(
          "importEdges",
          [](dsf::mobility::RoadNetwork& self,
             std::string const& fileName,
             char const separator) { self.importEdges(fileName, separator); },
          pybind11::arg("fileName"),
          pybind11::arg("separator"),
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::importEdges").c_str())
      .def(
          "importEdges",
          [](dsf::mobility::RoadNetwork& self,
             std::string const& fileName,
             bool const bCreateInverse) { self.importEdges(fileName, bCreateInverse); },
          pybind11::arg("fileName"),
          pybind11::arg("bCreateInverse"),
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::importEdges").c_str())
      .def(
          "importNodeProperties",
          [](dsf::mobility::RoadNetwork& self,
             std::string const& fileName,
             char const separator) { self.importNodeProperties(fileName, separator); },
          pybind11::arg("fileName"),
          pybind11::arg("separator") = ';',
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::importNodeProperties").c_str())
      .def(
          "importTrafficLights",
          &dsf::mobility::RoadNetwork::importTrafficLights,
          pybind11::arg("fileName"),
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::importTrafficLights").c_str())
      .def(
          "makeRoundabout",
          [](dsf::mobility::RoadNetwork& self, dsf::Id id) -> void {
            self.makeRoundabout(id);
          },
          pybind11::arg("id"),
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::makeRoundabout").c_str())
      .def(
          "makeTrafficLight",
          [](dsf::mobility::RoadNetwork& self,
             dsf::Id id,
             dsf::Delay const cycleTime,
             dsf::Delay const counter) -> void {
            self.makeTrafficLight(id, cycleTime, counter);
          },
          pybind11::arg("id"),
          pybind11::arg("cycleTime"),
          pybind11::arg("counter"),
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::makeTrafficLight").c_str())
      .def(
          "setStreetStatusById",
          &dsf::mobility::RoadNetwork::setStreetStatusById,
          pybind11::arg("streetId"),
          pybind11::arg("status"),
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::setStreetStatusById").c_str())
      .def("setStreetStatusByName",
           &dsf::mobility::RoadNetwork::setStreetStatusByName,
           pybind11::arg("name"),
           pybind11::arg("status"),
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::setStreetStatusByName")
               .c_str())
      .def("changeStreetNLanesById",
           &dsf::mobility::RoadNetwork::changeStreetNLanesById,
           pybind11::arg("streetId"),
           pybind11::arg("nLanes"),
           pybind11::arg("speedFactor") = std::nullopt,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::changeStreetNLanesById")
               .c_str())
      .def("changeStreetNLanesByName",
           &dsf::mobility::RoadNetwork::changeStreetNLanesByName,
           pybind11::arg("name"),
           pybind11::arg("nLanes"),
           pybind11::arg("speedFactor") = std::nullopt,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::changeStreetNLanesByName")
               .c_str())
      .def("changeStreetCapacityById",
           &dsf::mobility::RoadNetwork::changeStreetCapacityById,
           pybind11::arg("streetId"),
           pybind11::arg("factor"),
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::changeStreetCapacityById")
               .c_str())
      .def("changeStreetCapacityByName",
           &dsf::mobility::RoadNetwork::changeStreetCapacityByName,
           pybind11::arg("name"),
           pybind11::arg("factor"),
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::changeStreetCapacityByName")
               .c_str())
      .def("addCoil",
           &dsf::mobility::RoadNetwork::addCoil,
           pybind11::arg("streetId"),
           pybind11::arg("name") = std::string(),
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::addCoil").c_str())
      .def(
          "shortestPath",
          [](const dsf::mobility::RoadNetwork& self,
             dsf::Id sourceId,
             dsf::Id targetId,
             dsf::PathWeight weightFunction,
             double threshold) {
            return self.shortestPath(
                sourceId,
                targetId,
                [weightFunction](const dsf::mobility::Street& street) {
                  switch (weightFunction) {
                    case dsf::PathWeight::LENGTH:
                      return street.length();
                    case dsf::PathWeight::TRAVELTIME:
                      return street.length() / street.maxSpeed();
                    case dsf::PathWeight::WEIGHT:
                      return street.weight();
                    default:
                      return street.length() / street.maxSpeed();
                  }
                },
                threshold);
          },
          pybind11::arg("sourceId"),
          pybind11::arg("targetId"),
          pybind11::arg("weightFunction") = dsf::PathWeight::TRAVELTIME,
          pybind11::arg("threshold") = 1e-9,
          "Find the shortest path between two nodes using Dijkstra's algorithm.\n\n"
          "Args:\n"
          "    sourceId (int): The id of the source node\n"
          "    targetId (int): The id of the target node\n"
          "    weightFunction (PathWeight): The weight function to use (LENGTH, "
          "TRAVELTIME, or WEIGHT)\n"
          "    threshold (float): Relative tolerance applied to the full "
          "source-to-target path cost\n\n"
          "Returns:\n"
          "    PathCollection: A map where each key is a node id and the value is a "
          "vector of next hop node ids toward the target")
      .def(
          "computeBetweennessCentralities",
          [](dsf::mobility::RoadNetwork& self, const std::string& weight) {
            auto weightFunc = [&weight](const dsf::mobility::Street& street) {
              if (weight == "length") {
                return street.length();
              } else if (weight == "traveltime") {
                return street.length() / street.maxSpeed();
              } else if (weight == "weight") {
                return street.weight();
              } else {
                throw std::invalid_argument(
                    "Invalid weight function: '" + weight +
                    "'. Valid options are: 'length', 'traveltime', 'weight'.");
              }
            };
            self.computeBetweennessCentralities(weightFunc);
          },
          pybind11::arg("weight") = "length",
          "Compute betweenness centralities for all nodes using Brandes' algorithm.\n\n"
          "Args:\n"
          "    weight (str): The weight function to use. Options are:\n"
          "        - 'length': Use the street length as weight\n"
          "        - 'traveltime': Use length / max_speed as weight\n"
          "        - 'weight': Use the custom edge weight\n\n"
          "The results are stored in each node's betweennessCentrality attribute.")
      .def(
          "computeEdgeBetweennessCentralities",
          [](dsf::mobility::RoadNetwork& self, const std::string& weight) {
            auto weightFunc = [&weight](const dsf::mobility::Street& street) {
              if (weight == "length") {
                return street.length();
              } else if (weight == "traveltime") {
                return street.length() / street.maxSpeed();
              } else if (weight == "weight") {
                return street.weight();
              } else {
                throw std::invalid_argument(
                    "Invalid weight function: '" + weight +
                    "'. Valid options are: 'length', 'traveltime', 'weight'.");
              }
            };
            self.computeEdgeBetweennessCentralities(weightFunc);
          },
          pybind11::arg("weight") = "length",
          "Compute edge betweenness centralities for all edges using Brandes' "
          "algorithm.\n\n"
          "Args:\n"
          "    weight (str): The weight function to use. Options are:\n"
          "        - 'length': Use the street length as weight\n"
          "        - 'traveltime': Use length / max_speed as weight\n"
          "        - 'weight': Use the custom edge weight\n\n"
          "The results are stored in each edge's betweennessCentrality attribute.")
      .def(
          "nodeBetweennessCentralities",
          [](const dsf::mobility::RoadNetwork& self) {
            std::unordered_map<dsf::Id, std::optional<double>> result;
            for (auto const& [nodeId, pNode] : self.nodes()) {
              result[nodeId] = pNode->betweennessCentrality();
            }
            return result;
          },
          "Get the betweenness centrality values for all nodes.\n\n"
          "Returns:\n"
          "    dict[int, float | None]: A dictionary mapping node id to its "
          "betweenness centrality value (None if not computed).")
      .def(
          "edgeBetweennessCentralities",
          [](const dsf::mobility::RoadNetwork& self) {
            std::unordered_map<dsf::Id, std::optional<double>> result;
            for (auto const& [edgeId, pEdge] : self.edges()) {
              result[edgeId] = pEdge->betweennessCentrality();
            }
            return result;
          },
          "Get the betweenness centrality values for all edges.\n\n"
          "Returns:\n"
          "    dict[int, float | None]: A dictionary mapping edge id to its "
          "betweenness centrality value (None if not computed).");

  pybind11::class_<dsf::mobility::PathCollection>(mobility, "PathCollection")
      .def(pybind11::init<>(), "Create an empty PathCollection")
      .def(
          "__getitem__",
          [](const dsf::mobility::PathCollection& self, dsf::Id key) {
            auto it = self.find(key);
            if (it == self.end()) {
              throw pybind11::key_error("Key not found");
            }
            return it->second;
          },
          pybind11::arg("key"),
          "Get the next hops for a given node id")
      .def(
          "__setitem__",
          [](dsf::mobility::PathCollection& self,
             dsf::Id key,
             std::vector<dsf::Id> value) { self[key] = value; },
          pybind11::arg("key"),
          pybind11::arg("value"),
          "Set the next hops for a given node id")
      .def(
          "__contains__",
          [](const dsf::mobility::PathCollection& self, dsf::Id key) {
            return self.find(key) != self.end();
          },
          pybind11::arg("key"),
          "Check if a node id exists in the collection")
      .def(
          "__len__",
          [](const dsf::mobility::PathCollection& self) { return self.size(); },
          "Get the number of nodes in the collection")
      .def(
          "keys",
          [](const dsf::mobility::PathCollection& self) {
            std::vector<dsf::Id> keys;
            keys.reserve(self.size());
            for (const auto& [key, _] : self) {
              keys.push_back(key);
            }
            return keys;
          },
          "Get all node ids in the collection")
      .def(
          "items",
          [](const dsf::mobility::PathCollection& self) {
            pybind11::dict items;
            for (const auto& [key, value] : self) {
              items[pybind11::int_(key)] = pybind11::cast(value);
            }
            return items;
          },
          "Get all items (node id, next hops) in the collection")
      .def("explode",
           &dsf::mobility::PathCollection::explode,
           pybind11::arg("sourceId"),
           pybind11::arg("targetId"),
           dsf::g_docstrings.at("dsf::mobility::PathCollection::explode").c_str());

  pybind11::class_<dsf::mobility::Itinerary>(mobility, "Itinerary")
      .def(pybind11::init<dsf::Id, dsf::Id>(),
           pybind11::arg("id"),
           pybind11::arg("destination"),
           dsf::g_docstrings.at("dsf::mobility::Itinerary::Itinerary").c_str())
      .def("setPath",
           &dsf::mobility::Itinerary::setPath,
           pybind11::arg("path"),
           dsf::g_docstrings.at("dsf::mobility::Itinerary::setPath").c_str())
      .def("id",
           &dsf::mobility::Itinerary::id,
           dsf::g_docstrings.at("dsf::mobility::Itinerary::id").c_str())
      .def("destination",
           &dsf::mobility::Itinerary::destination,
           dsf::g_docstrings.at("dsf::mobility::Itinerary::destination").c_str());
  // .def("path", &dsf::mobility::Itinerary::path, pybind11::return_value_policy::reference_internal);

  pybind11::class_<dsf::mobility::FirstOrderDynamics>(mobility, "Dynamics")
      //     // Constructors are not directly exposed due to the template nature and complexity.
      //     // Users should use derived classes like FirstOrderDynamics.
      .def(
          pybind11::init<dsf::mobility::RoadNetwork&, bool, std::optional<unsigned int>>(),
          pybind11::arg("graph"),
          pybind11::arg("useCache") = false,
          pybind11::arg("seed") = std::nullopt,
          pybind11::keep_alive<1, 2>(),
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::FirstOrderDynamics")
              .c_str())
      .def(
          "setSpeedFunction",
          [](dsf::mobility::FirstOrderDynamics& self,
             dsf::SpeedFunction speedFunction,
             pybind11::object arg) {
            switch (speedFunction) {
              case dsf::SpeedFunction::LINEAR:
                self.setSpeedFunction(dsf::SpeedFunction::LINEAR, arg.cast<double>());
                break;
              case dsf::SpeedFunction::CUSTOM: {
                // Numba cfunc path: raw C function pointer, zero Python overhead
                auto* func_ptr =
                    reinterpret_cast<double (*)(double, double)>(arg.cast<uintptr_t>());
                self.setSpeedFunction(
                    dsf::SpeedFunction::CUSTOM,
                    [func_ptr](dsf::mobility::Street const& street) -> double {
                      return func_ptr(street.maxSpeed(), street.density(true));
                    });
                break;
              }
              default:
                throw std::invalid_argument("Invalid speed function type");
            }
          },
          pybind11::arg("speedFunction"),
          pybind11::arg("arg"),
          "Set the speed function for agents.\n\n"
          "Args:\n"
          "    speedFunction (SpeedFunction): The speed function type (LINEAR or "
          "CUSTOM)\n"
          "    arg: For LINEAR, a float alpha in [0., 1.). "
          "For CUSTOM, an integer address (uintptr_t) of a C function with signature "
          "double(double max_speed, double density).")
      .def("setName",
           &dsf::mobility::FirstOrderDynamics::setName,
           pybind11::arg("name"),
           dsf::g_docstrings.at("dsf::Dynamics::setName").c_str())
      .def("setInitTime",
           &dsf::mobility::FirstOrderDynamics::setInitTime,
           pybind11::arg("timeEpoch"),
           dsf::g_docstrings.at("dsf::Dynamics::setInitTime").c_str())
      .def(
          "setInitTime",
          [](dsf::mobility::FirstOrderDynamics& self, pybind11::object datetime_obj) {
            auto const epoch_seconds =
                pybind11::cast<double>(datetime_obj.attr("timestamp")());
            self.setInitTime(static_cast<std::time_t>(epoch_seconds));
          },
          pybind11::arg("datetime"),
          dsf::g_docstrings.at("dsf::Dynamics::setInitTime").c_str())
      .def("setConcurrency",
           &dsf::mobility::FirstOrderDynamics::setConcurrency,
           pybind11::arg("concurrency"),
           dsf::g_docstrings.at("dsf::Dynamics::setConcurrency").c_str())
      .def("connectDataBase",
           &dsf::mobility::FirstOrderDynamics::connectDataBase,
           pybind11::arg("dbPath"),
           pybind11::arg("queries") =
               "PRAGMA busy_timeout = 5000;PRAGMA journal_mode = WAL;PRAGMA "
               "synchronous=NORMAL;PRAGMA temp_store=MEMORY;PRAGMA cache_size=-20000;",
           dsf::g_docstrings.at("dsf::Dynamics::connectDataBase").c_str())
      .def("setForcePriorities",
           &dsf::mobility::FirstOrderDynamics::setForcePriorities,
           pybind11::arg("forcePriorities"),
           dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::setForcePriorities")
               .c_str())
      .def(
          "setDataUpdatePeriod",
          [](dsf::mobility::FirstOrderDynamics& self, int dataUpdatePeriod) {
            self.setDataUpdatePeriod(static_cast<dsf::Delay>(dataUpdatePeriod));
          },
          pybind11::arg("dataUpdatePeriod"),
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::setDataUpdatePeriod")
              .c_str())
      .def(
          "setMeanTravelDistance",
          &dsf::mobility::FirstOrderDynamics::setMeanTravelDistance,
          pybind11::arg("meanDistance"),
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::setMeanTravelDistance")
              .c_str())
      .def(
          "setMeanTravelTime",
          [](dsf::mobility::FirstOrderDynamics& self, uint64_t meanTravelTime) {
            self.setMeanTravelTime(static_cast<std::time_t>(meanTravelTime));
          },
          pybind11::arg("meanTravelTime"),
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::setMeanTravelTime")
              .c_str())
      .def("setErrorProbability",
           &dsf::mobility::FirstOrderDynamics::setErrorProbability,
           pybind11::arg("errorProbability"),
           dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::setErrorProbability")
               .c_str())
      .def("setWeightFunction",
           &dsf::mobility::FirstOrderDynamics::setWeightFunction,
           pybind11::arg("weightFunction"),
           pybind11::arg("weightThreshold") = std::nullopt)
      .def("killStagnantAgents",
           &dsf::mobility::FirstOrderDynamics::killStagnantAgents,
           pybind11::arg("timeToleranceFactor") = 3.,
           dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::killStagnantAgents")
               .c_str())
      .def(
          "setDestinationNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             const std::vector<dsf::Id>& destinationNodes) {
            self.setDestinationNodes(destinationNodes);
          },
          pybind11::arg("destinationNodes"),
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::setDestinationNodes")
              .c_str())
      .def(
          "setOriginNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             const std::unordered_map<dsf::Id, double>& originNodes) {
            self.setOriginNodes(originNodes);
          },
          pybind11::arg("originNodes") = std::unordered_map<dsf::Id, double>(),
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::setOriginNodes")
              .c_str())
      .def(
          "setOriginNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             pybind11::array_t<dsf::Id> originNodes) {
            // Convert numpy array to vector with equal weights
            auto buf = originNodes.request();
            auto* ptr = static_cast<dsf::Id*>(buf.ptr);
            std::unordered_map<dsf::Id, double> nodeWeights;
            for (size_t i = 0; i < buf.size; ++i) {
              nodeWeights[ptr[i]] = 1.0;  // Equal weight for all nodes
            }
            self.setOriginNodes(nodeWeights);
          },
          pybind11::arg("originNodes"),
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::setOriginNodes")
              .c_str())
      .def(
          "setDestinationNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             pybind11::array_t<dsf::Id> destinationNodes) {
            // Convert numpy array to vector
            auto buf = destinationNodes.request();
            auto* ptr = static_cast<dsf::Id*>(buf.ptr);
            std::vector<dsf::Id> nodes(ptr, ptr + buf.size);
            self.setDestinationNodes(nodes);
          },
          pybind11::arg("destinationNodes"),
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::setDestinationNodes")
              .c_str())
      .def(
          "setDestinationNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             const std::unordered_map<dsf::Id, double>& destinationNodes) {
            self.setDestinationNodes(destinationNodes);
          },
          pybind11::arg("destinationNodes"),
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::setDestinationNodes")
              .c_str())
      .def("setODs",
           &dsf::mobility::FirstOrderDynamics::setODs,
           pybind11::arg("ods"),
           dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::setODs").c_str())
      .def("initTurnCounts",
           &dsf::mobility::FirstOrderDynamics::initTurnCounts,
           dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::initTurnCounts")
               .c_str())
      .def("updatePaths",
           &dsf::mobility::FirstOrderDynamics::updatePaths,
           pybind11::arg("throw_on_empty") = true,
           dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::updatePaths").c_str())
      .def("addAgentsUniformly",
           &dsf::mobility::FirstOrderDynamics::addAgentsUniformly,
           pybind11::arg("nAgents"),
           pybind11::arg("itineraryId") = std::nullopt,
           dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::addAgentsUniformly")
               .c_str())
      .def(
          "addAgents",
          [](dsf::mobility::FirstOrderDynamics& self,
             std::size_t nAgents,
             dsf::mobility::AgentInsertionMethod insertionMethod) {
            self.addAgents(nAgents, insertionMethod);
          },
          pybind11::arg("nAgents"),
          pybind11::arg("insertionMethod"),
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::addAgents").c_str())
      .def("evolve",
           &dsf::mobility::FirstOrderDynamics::evolve,
           pybind11::arg("reinsert_agents") = false,
           dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::evolve").c_str())
      .def(
          "optimizeTrafficLights",
          &dsf::mobility::FirstOrderDynamics::optimizeTrafficLights,
          pybind11::arg("optimizationType") = dsf::TrafficLightOptimization::DOUBLE_TAIL,
          pybind11::arg("logFile") = "",
          pybind11::arg("threshold") = 0.,
          pybind11::arg("ratio") = 1.3,
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::optimizeTrafficLights")
              .c_str())
      .def(
          "graph",
          [](dsf::mobility::FirstOrderDynamics& self) -> dsf::mobility::RoadNetwork& {
            return self.graph();
          },
          pybind11::return_value_policy::reference_internal,
          dsf::g_docstrings.at("dsf::Dynamics::graph").c_str())
      .def("nAgents",
           &dsf::mobility::FirstOrderDynamics::nAgents,
           dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::nAgents").c_str())
      .def("time",
           &dsf::mobility::FirstOrderDynamics::time,
           dsf::g_docstrings.at("dsf::Dynamics::time").c_str())
      .def("time_step",
           &dsf::mobility::FirstOrderDynamics::time_step,
           dsf::g_docstrings.at("dsf::Dynamics::time_step").c_str())
      .def("datetime",
           &dsf::mobility::FirstOrderDynamics::strDateTime,
           dsf::g_docstrings.at("dsf::Dynamics::strDateTime").c_str())
      .def("meanTravelTime",
           &dsf::mobility::FirstOrderDynamics::meanTravelTime,
           pybind11::arg("clearData") = false,
           dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::meanTravelTime")
               .c_str())
      .def("meanTravelDistance",
           &dsf::mobility::FirstOrderDynamics::meanTravelDistance,
           pybind11::arg("clearData") = false,
           dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::meanTravelDistance")
               .c_str())
      .def("meanTravelSpeed",
           &dsf::mobility::FirstOrderDynamics::meanTravelSpeed,
           pybind11::arg("clearData") = false,
           dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::meanTravelSpeed")
               .c_str())
      .def(
          "turnCounts",
          [](const dsf::mobility::FirstOrderDynamics& self) {
            // Convert C++ unordered_map<Id, unordered_map<Id, size_t>> to Python dict of dicts
            pybind11::dict py_result;
            for (const auto& [from_id, inner_map] : self.turnCounts()) {
              pybind11::dict py_inner;
              for (const auto& [to_id, count] : inner_map) {
                py_inner[pybind11::int_(to_id)] = pybind11::int_(count);
              }
              py_result[pybind11::int_(from_id)] = py_inner;
            }
            return py_result;
          },
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::turnCounts").c_str())
      .def(
          "normalizedTurnCounts",
          [](const dsf::mobility::FirstOrderDynamics& self) {
            // Convert C++ unordered_map<Id, unordered_map<Id, size_t>> to Python dict of dicts
            pybind11::dict py_result;
            for (const auto& [from_id, inner_map] : self.normalizedTurnCounts()) {
              pybind11::dict py_inner;
              for (const auto& [to_id, count] : inner_map) {
                py_inner[pybind11::int_(to_id)] = pybind11::float_(count);
              }
              py_result[pybind11::int_(from_id)] = py_inner;
            }
            return py_result;
          },
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::normalizedTurnCounts")
              .c_str())
      .def(
          "originCounts",
          [](dsf::mobility::FirstOrderDynamics& self, bool reset) {
            // Convert C++ unordered_map<Id, size_t> to Python dict
            pybind11::dict py_result;
            for (const auto& [node_id, count] : self.originCounts(reset)) {
              py_result[pybind11::int_(node_id)] = pybind11::int_(count);
            }
            return py_result;
          },
          pybind11::arg("reset") = true,
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::originCounts").c_str())
      .def(
          "destinationCounts",
          [](dsf::mobility::FirstOrderDynamics& self, bool reset) {
            // Convert C++ unordered_map<Id, size_t> to Python dict
            pybind11::dict py_result;
            for (const auto& [node_id, count] : self.destinationCounts(reset)) {
              py_result[pybind11::int_(node_id)] = pybind11::int_(count);
            }
            return py_result;
          },
          pybind11::arg("reset") = true,
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::destinationCounts")
              .c_str())
      .def("saveData",
           &dsf::mobility::FirstOrderDynamics::saveData,
           pybind11::arg("saving_interval"),
           pybind11::arg("save_average_stats") = false,
           pybind11::arg("save_street_data") = false,
           pybind11::arg("save_travel_data") = false,
           pybind11::arg("save_agent_data") = false,
           "Configure data saving during simulation.\n\n"
           "Args:\n"
           "    saving_interval: Interval in time steps between data saves\n"
           "    save_average_stats: Whether to save average statistics (speed, density, "
           "flow)\n"
           "    save_street_data: Whether to save per-street data (density, speed, coil "
           "counts)\n"
           "    save_travel_data: Whether to save travel data (distance, travel time)\n"
           "    save_agent_data: Whether to save per-agent edge traversal data")
      .def(
          "summary",
          [](dsf::mobility::FirstOrderDynamics& self) {
            self.summary();  // Uses default std::cout
          },
          dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::summary").c_str());

  // Bind TrajectoryCollection class to mdt submodule
  pybind11::class_<dsf::mdt::TrajectoryCollection>(mdt, "TrajectoryCollection")
      .def(pybind11::init<std::string const&,
                          std::unordered_map<std::string, std::string> const&,
                          char const,
                          std::array<double, 4> const&>(),
           pybind11::arg("fileName"),
           pybind11::arg("column_mapping") =
               std::unordered_map<std::string, std::string>{},
           pybind11::arg("separator") = ';',
           pybind11::arg("bbox") = std::array<double, 4>{},
           dsf::g_docstrings.at("dsf::mdt::TrajectoryCollection::TrajectoryCollection")
               .c_str())
      .def(
          pybind11::init([](pybind11::object df) {
            pybind11::object columns = df.attr("columns");
            pybind11::array arr = df.attr("to_numpy")();
            // Expect a 2D numpy array (rows x cols) and an iterable of column names
            auto info = arr.request();
            if (info.ndim != 2) {
              throw std::runtime_error(
                  "TrajectoryCollection constructor expects a 2D numpy array from "
                  "df.to_numpy()");
            }
            std::size_t n_rows = static_cast<std::size_t>(info.shape[0]);
            std::size_t n_cols = static_cast<std::size_t>(info.shape[1]);

            // Collect column names
            std::vector<std::string> colnames;
            for (auto item : columns) {
              colnames.push_back(pybind11::str(item));
            }

            // Build unordered_map<string, vector<string>> where each key is a column name
            std::unordered_map<std::string,
                               std::variant<std::vector<dsf::Id>,
                                            std::vector<std::time_t>,
                                            std::vector<double>>>
                dataframe;
            dataframe.reserve(n_cols);

            // Columns should be uid timestamp lat lon
            dataframe["uid"] = std::vector<dsf::Id>();
            std::get<std::vector<dsf::Id>>(dataframe.at("uid")).reserve(n_rows);
            dataframe["timestamp"] = std::vector<std::time_t>();
            std::get<std::vector<std::time_t>>(dataframe.at("timestamp")).reserve(n_rows);
            dataframe["lat"] = std::vector<double>();
            std::get<std::vector<double>>(dataframe.at("lat")).reserve(n_rows);
            dataframe["lon"] = std::vector<double>();
            std::get<std::vector<double>>(dataframe.at("lon")).reserve(n_rows);

            for (auto const& colname : colnames) {
              if (colname == "uid") {
                for (std::size_t i = 0; i < n_rows; ++i) {
                  pybind11::object cell = arr[pybind11::make_tuple(i, 0)];
                  std::get<std::vector<dsf::Id>>(dataframe.at("uid"))
                      .push_back(static_cast<dsf::Id>(pybind11::cast<double>(cell)));
                }
              } else if (colname == "timestamp") {
                for (std::size_t i = 0; i < n_rows; ++i) {
                  pybind11::object cell = arr[pybind11::make_tuple(i, 1)];
                  std::get<std::vector<std::time_t>>(dataframe.at("timestamp"))
                      .push_back(static_cast<std::time_t>(pybind11::cast<double>(cell)));
                }
              } else if (colname == "lat") {
                for (std::size_t i = 0; i < n_rows; ++i) {
                  pybind11::object cell = arr[pybind11::make_tuple(i, 2)];
                  std::get<std::vector<double>>(dataframe.at("lat"))
                      .push_back(pybind11::cast<double>(cell));
                }
              } else if (colname == "lon") {
                for (std::size_t i = 0; i < n_rows; ++i) {
                  pybind11::object cell = arr[pybind11::make_tuple(i, 3)];
                  std::get<std::vector<double>>(dataframe.at("lon"))
                      .push_back(pybind11::cast<double>(cell));
                }
              }
            }

            return new dsf::mdt::TrajectoryCollection(std::move(dataframe));
          }),
          pybind11::arg("df"),
          // Write this docstring manually as it is not in g_docstrings
          "Constructor that builds a TrajectoryCollection from a pandas or polars "
          "DataFrame.\n\nArgs:\n\tdf (pandas.DataFrame | polars.DataFrame): Input "
          "DataFrame. Must contain the following columns:\n\t\t'uid' (identifier), "
          "'timestamp' (epoch seconds), 'lat' (latitude),\n\t\t'lon' (longitude). The "
          "constructor will call ``df.columns`` and\n\t\t``df.to_numpy()`` internally. "
          "All cell values are converted to strings\n\t\twhen building the underlying "
          "C++ data structure.\n\nReturns:\n\tdsf.mdt.TrajectoryCollection: A new "
          "TrajectoryCollection constructed from\n\tthe provided DataFrame.")
      .def("filter",
           &dsf::mdt::TrajectoryCollection::filter,
           pybind11::arg("cluster_radius_km"),
           pybind11::arg("max_speed_kph") = 150.0,
           pybind11::arg("min_points_per_trajectory") = 2,
           pybind11::arg("min_duration_min") = pybind11::none(),
           dsf::g_docstrings.at("dsf::mdt::TrajectoryCollection::filter").c_str())
      .def("to_csv",
           &dsf::mdt::TrajectoryCollection::to_csv,
           pybind11::arg("fileName"),
           pybind11::arg("sep") = ';',
           dsf::g_docstrings.at("dsf::mdt::TrajectoryCollection::to_csv").c_str())
      .def(
          "to_pandas",
          [](const dsf::mdt::TrajectoryCollection& self) {
            // Convert the internal data to a pandas DataFrame
            pybind11::module_ pd = pybind11::module_::import("pandas");
            pybind11::dict data_dict;

            // Prepare columns
            std::vector<dsf::Id> uids;
            std::vector<std::size_t> trajectoryIds;
            std::vector<double> lons;
            std::vector<double> lats;
            std::vector<std::time_t> timestamps_in;
            std::vector<std::time_t> timestamps_out;

            for (auto const& [uid, trajectories] : self.trajectories()) {
              std::size_t trajIdx = 0;
              for (auto const& trajectory : trajectories) {
                for (auto const& cluster : trajectory.points()) {
                  auto const centroid = cluster.centroid();
                  uids.push_back(uid);
                  trajectoryIds.push_back(trajIdx);
                  lons.push_back(centroid.x());
                  lats.push_back(centroid.y());
                  timestamps_in.push_back(cluster.firstTimestamp());
                  timestamps_out.push_back(cluster.lastTimestamp());
                }
                ++trajIdx;
              }
            }

            data_dict["uid"] = pybind11::array(uids.size(), uids.data());
            data_dict["trajectory_id"] =
                pybind11::array(trajectoryIds.size(), trajectoryIds.data());

            data_dict["lon"] = pybind11::array(lons.size(), lons.data());
            data_dict["lat"] = pybind11::array(lats.size(), lats.data());
            data_dict["timestamp_in"] =
                pybind11::array(timestamps_in.size(), timestamps_in.data());
            data_dict["timestamp_out"] =
                pybind11::array(timestamps_out.size(), timestamps_out.data());

            return pd.attr("DataFrame")(data_dict);
          },
          "Convert the TrajectoryCollection to a pandas DataFrame.\n\nReturns:\n\tpandas."
          "DataFrame: DataFrame containing the trajectory data with columns 'uid', "
          "'trajectory_id', 'lon', 'lat', 'timestamp_in', and 'timestamp_out'.")
      .def(
          "to_polars",
          [](const dsf::mdt::TrajectoryCollection& self) {
            // Convert the internal data to a polars DataFrame
            pybind11::module_ pl = pybind11::module_::import("polars");
            pybind11::dict data_dict;

            // Prepare columns
            std::vector<dsf::Id> uids;
            std::vector<std::size_t> trajectoryIds;
            std::vector<double> lons;
            std::vector<double> lats;
            std::vector<std::time_t> timestamps_in;
            std::vector<std::time_t> timestamps_out;

            for (auto const& [uid, trajectories] : self.trajectories()) {
              std::size_t trajIdx = 0;
              for (auto const& trajectory : trajectories) {
                for (auto const& cluster : trajectory.points()) {
                  auto const centroid = cluster.centroid();
                  uids.push_back(uid);
                  trajectoryIds.push_back(trajIdx);
                  lons.push_back(centroid.x());
                  lats.push_back(centroid.y());
                  timestamps_in.push_back(cluster.firstTimestamp());
                  timestamps_out.push_back(cluster.lastTimestamp());
                }
                ++trajIdx;
              }
            }

            data_dict["uid"] = pybind11::array(uids.size(), uids.data());
            data_dict["trajectory_id"] =
                pybind11::array(trajectoryIds.size(), trajectoryIds.data());

            data_dict["lon"] = pybind11::array(lons.size(), lons.data());
            data_dict["lat"] = pybind11::array(lats.size(), lats.data());
            data_dict["timestamp_in"] =
                pybind11::array(timestamps_in.size(), timestamps_in.data());
            data_dict["timestamp_out"] =
                pybind11::array(timestamps_out.size(), timestamps_out.data());

            return pl.attr("DataFrame")(data_dict);
          },
          "Convert the TrajectoryCollection to a polars DataFrame.\n\nReturns:\n\tpolars."
          "DataFrame: DataFrame containing the trajectory data with columns 'uid', "
          "'trajectory_id', 'lon', 'lat', 'timestamp_in', and 'timestamp_out'.");
}
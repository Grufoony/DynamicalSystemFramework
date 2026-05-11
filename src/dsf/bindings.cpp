#include "dsf.hpp"

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
      .def("id",
           &dsf::mobility::Street::id,
           R"doc(Return the unique identifier of this street.

    Returns:
      int: The street id.)doc")
      .def("source",
           &dsf::mobility::Street::source,
           R"doc(Get the source node id of this street.

    Returns:
        int: Source node id.)doc")
      .def("target",
           &dsf::mobility::Street::target,
           R"doc(Get the target node id of this street.

    Returns:
        int: Target node id.)doc")
      .def("geometry",
           &dsf::mobility::Street::geometry,
           R"doc(Get the geometry object for this street.

    Returns:
        object: Geometry representation (library-specific).)doc")
      .def("name",
           &dsf::mobility::Street::name,
           R"doc(Get the name of the street.

    Returns:
        str: The street's name (empty if unnamed).)doc")
      .def("length",
           &dsf::mobility::Street::length,
           R"doc(Get the length of the street.

    Returns:
        float: Length in metres.)doc")
      .def("nLanes",
           &dsf::mobility::Street::nLanes,
           R"doc(Get the number of lanes on this street.

    Returns:
        int: Number of lanes.)doc")
      .def("maxSpeed",
           &dsf::mobility::Street::maxSpeed,
           R"doc(Get the maximum permitted speed on this street.

    Returns:
        float: Maximum speed (units depend on network configuration).)doc")
      .def("capacity",
           &dsf::mobility::Street::capacity,
           R"doc(Get the capacity of this street.

    Returns:
        float: Capacity value (units: vehicles).)doc")
      .def("transportCapacity",
           &dsf::mobility::Street::transportCapacity,
           R"doc(Get the transport capacity of this street.

    Returns:
        float: Transport capacity (units: vehicles or payload).)doc")
      .def("roadStatus",
           &dsf::mobility::Street::roadStatus,
           R"doc(Get the current status of the road.

    Returns:
        RoadStatus: Enum value indicating OPEN or CLOSED.)doc")
      .def("estimatedTravelTime",
           &dsf::mobility::Street::estimatedTravelTime,
           "Get estimated travel time for this street using the active estimator.")
      .def("attributes",
           &dsf::mobility::Street::attributes,
           R"doc(Get the attribute dictionary for this street.

    Returns:
        dict: Mapping of attribute names to values.)doc");

  // Bind RoadJunction class to mobility submodule
  pybind11::class_<dsf::mobility::RoadJunction>(mobility, "RoadJunction")
      .def("id",
           &dsf::mobility::RoadJunction::id,
           R"doc(Return the unique identifier for this junction.

    Returns:
        int: Junction id.)doc")
      .def("geometry",
           &dsf::mobility::RoadJunction::geometry,
           R"doc(Get the geometry of the junction.

    Returns:
        object: Geometry representation.)doc")
      .def("capacity",
           &dsf::mobility::RoadJunction::capacity,
           R"doc(Get the capacity of the junction.

    Returns:
        float: Capacity value.)doc")
      .def("transportCapacity",
           &dsf::mobility::RoadJunction::transportCapacity,
           R"doc(Get the transport capacity of the junction.

    Returns:
        float: Transport capacity.)doc")
      .def("attributes",
           &dsf::mobility::RoadJunction::attributes,
           R"doc(Get the attribute dictionary for this junction.

    Returns:
        dict: Mapping of attribute names to values.)doc");

  // Bind Measurement to main module (can be used across different contexts)
  pybind11::class_<dsf::Measurement<double>>(m, "Measurement")
      .def(pybind11::init<double, double, std::size_t>(),
           pybind11::arg("mean"),
           pybind11::arg("std"),
           pybind11::arg("n"),
           R"doc(Create a Measurement object.

    Args:
      mean (float): Sample mean.
      std (float): Sample standard deviation.
      n (int): Number of samples.

    Returns:
      Measurement: Initialized measurement instance.)doc")
      .def_readwrite("mean",
                     &dsf::Measurement<double>::mean,
                     R"doc(Mean value of the measurement.

                Type:
                  float)doc")
      .def_readwrite("std",
                     &dsf::Measurement<double>::std,
                     R"doc(Standard deviation of the measurement.

          Type:
            float)doc")
      .def_readwrite("n",
                     &dsf::Measurement<double>::n,
                     R"doc(Number of samples used to compute the measurement.

          Type:
            int)doc")
      .def_readwrite("is_valid",
                     &dsf::Measurement<double>::is_valid,
                     R"doc(Flag indicating whether the measurement is valid.

          Type:
            bool)doc");

  // Bind mobility-related classes to mobility submodule
  pybind11::class_<dsf::mobility::RoadNetwork>(mobility, "RoadNetwork")
      .def(pybind11::init<>(),
           R"doc(Create an empty RoadNetwork instance.

    Returns:
        RoadNetwork: New network object.)doc")
      .def("nNodes",
           &dsf::mobility::RoadNetwork::nNodes,
           R"doc(Get the number of nodes in the network.

      Returns:
        int: Count of nodes.)doc")
      .def("nEdges",
           &dsf::mobility::RoadNetwork::nEdges,
           R"doc(Get the number of edges in the network.

      Returns:
        int: Count of edges.)doc")
      .def("nCoils",
           &dsf::mobility::RoadNetwork::nCoils,
           R"doc(Get the number of coils in the network.

      Returns:
        int: Count of coils.)doc")
      .def("nIntersections",
           &dsf::mobility::RoadNetwork::nIntersections,
           R"doc(Get the number of intersections in the network.

      Returns:
        int: Count of intersections.)doc")
      .def("nRoundabouts",
           &dsf::mobility::RoadNetwork::nRoundabouts,
           R"doc(Get the number of roundabouts in the network.

      Returns:
        int: Count of roundabouts.)doc")
      .def("nTrafficLights",
           &dsf::mobility::RoadNetwork::nTrafficLights,
           R"doc(Get the number of traffic lights in the network.

      Returns:
        int: Count of traffic lights.)doc")
      // Bind node and edge Network accessors, which return a ref or a cost ref
      // node should return a RoadJunction and edge should return a Street
      .def(
          "node",
          [](dsf::mobility::RoadNetwork& self, dsf::Id nodeId)
              -> dsf::mobility::RoadJunction& { return self.node(nodeId); },
          pybind11::arg("nodeId"),
          pybind11::return_value_policy::reference_internal,
          R"doc(Get a reference to the node with the given id.

      Args:
        nodeId (int): Node identifier.

      Returns:
        RoadJunction: Reference to the requested junction.)doc")
      .def(
          "edge",
          [](dsf::mobility::RoadNetwork& self, dsf::Id edgeId) -> dsf::mobility::Street& {
            return self.edge(edgeId);
          },
          pybind11::arg("edgeId"),
          pybind11::return_value_policy::reference_internal,
          R"doc(Get a reference to the edge with the given id.

      Args:
        edgeId (int): Edge identifier.

      Returns:
        Street: Reference to the requested street.)doc")
      .def("capacity",
           &dsf::mobility::RoadNetwork::capacity,
           R"doc(Get the capacity value for the network or a specific resource.

      Returns:
        float: Capacity.)doc")
      .def("adjustNodeCapacities",
           &dsf::mobility::RoadNetwork::adjustNodeCapacities,
           R"doc(Adjust capacities of nodes according to provided factors.

      Args:
        (See C++ API) Adjusts node capacities in-place.)doc")
      .def("autoInitTrafficLights",
           &dsf::mobility::RoadNetwork::autoInitTrafficLights,
           pybind11::arg("mainRoadPercentage") = 0.6,
           R"doc(Auto-initialize traffic lights based on network heuristics.

      Args:
        mainRoadPercentage (float, optional): Fraction used to identify main roads. Default 0.6)doc")
      .def(
          "autoMapStreetLanes",
          &dsf::mobility::RoadNetwork::autoMapStreetLanes,
          R"doc(Auto-map lanes for streets using heuristics based on geometry and attributes.)doc")
      .def("setEdgeWeight",
           &dsf::mobility::RoadNetwork::setEdgeWeight,
           pybind11::arg("weight"),
           pybind11::arg("threshold") = std::nullopt,
           R"doc(Set edge weights for routing and analysis.

      Args:
        weight (float): Weight value.
        threshold (float | None): Optional threshold to apply.)doc")
      .def(
          "describe",
          [](dsf::mobility::RoadNetwork& self) {
            self.describe();  // Uses default std::cout
          },
          R"doc(Write a textual description of the network to stdout.

      Returns:
        None)doc")
      .def("autoAssignRoadPriorities",
           &dsf::mobility::RoadNetwork::autoAssignRoadPriorities,
           R"doc(Auto-assign priorities to roads based on topology and attributes.)doc")
      .def(
          "importEdges",
          [](dsf::mobility::RoadNetwork& self, const std::string& fileName) {
            self.importEdges(fileName);
          },
          pybind11::arg("fileName"),
          R"doc(Import edges from a CSV file.

      Args:
        fileName (str): Path to the CSV file.)doc")
      .def(
          "importEdges",
          [](dsf::mobility::RoadNetwork& self,
             std::string const& fileName,
             char const separator) { self.importEdges(fileName, separator); },
          pybind11::arg("fileName"),
          pybind11::arg("separator"),
          R"doc(Import edges from a CSV file with a custom separator.

      Args:
        fileName (str): Path to the CSV file.
        separator (str): Field separator character.)doc")
      .def(
          "importEdges",
          [](dsf::mobility::RoadNetwork& self,
             std::string const& fileName,
             bool const bCreateInverse) { self.importEdges(fileName, bCreateInverse); },
          pybind11::arg("fileName"),
          pybind11::arg("bCreateInverse"),
          R"doc(Import edges from a CSV file and optionally create inverse edges.

      Args:
        fileName (str): Path to the CSV file.
        bCreateInverse (bool): Create inverse edges when true.)doc")
      .def(
          "importNodeProperties",
          [](dsf::mobility::RoadNetwork& self,
             std::string const& fileName,
             char const separator) { self.importNodeProperties(fileName, separator); },
          pybind11::arg("fileName"),
          pybind11::arg("separator") = ';',
          R"doc(Import node properties from a CSV file.

      Args:
        fileName (str): Path to the CSV file.
        separator (str): Field separator character.)doc")
      .def("importTrafficLights",
           &dsf::mobility::RoadNetwork::importTrafficLights,
           pybind11::arg("fileName"),
           R"doc(Import traffic light configurations from a file.

      Args:
        fileName (str): Path to the configuration file.)doc")
      .def(
          "makeRoundabout",
          [](dsf::mobility::RoadNetwork& self, dsf::Id id) -> void {
            self.makeRoundabout(id);
          },
          pybind11::arg("id"),
          R"doc(Convert the node with the given id into a roundabout.

      Args:
        id (int): Node id to convert.)doc")
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
          R"doc(Create a traffic light at the given node id.

      Args:
        id (int): Node id.
        cycleTime (Delay): Cycle time for the traffic light.
        counter (Delay): Initial counter value.)doc")
      .def("setStreetStatusById",
           &dsf::mobility::RoadNetwork::setStreetStatusById,
           pybind11::arg("streetId"),
           pybind11::arg("status"),
           R"doc(Set the status of a street by id.

      Args:
        streetId (int): Street identifier.
        status (RoadStatus): New status to set.)doc")
      .def("setStreetStatusByName",
           &dsf::mobility::RoadNetwork::setStreetStatusByName,
           pybind11::arg("name"),
           pybind11::arg("status"),
           R"doc(Set the status of a street by name.

      Args:
        name (str): Street name.
        status (RoadStatus): New status to set.)doc")
      .def("changeStreetNLanesById",
           &dsf::mobility::RoadNetwork::changeStreetNLanesById,
           pybind11::arg("streetId"),
           pybind11::arg("nLanes"),
           pybind11::arg("speedFactor") = std::nullopt,
           R"doc(Change the number of lanes for a street by id.

      Args:
        streetId (int): Street identifier.
        nLanes (int): New number of lanes.
        speedFactor (float | None): Optional speed factor.)doc")
      .def("changeStreetNLanesByName",
           &dsf::mobility::RoadNetwork::changeStreetNLanesByName,
           pybind11::arg("name"),
           pybind11::arg("nLanes"),
           pybind11::arg("speedFactor") = std::nullopt,
           R"doc(Change the number of lanes for a street by name.

      Args:
        name (str): Street name.
        nLanes (int): New number of lanes.
        speedFactor (float | None): Optional speed factor.)doc")
      .def("changeStreetCapacityById",
           &dsf::mobility::RoadNetwork::changeStreetCapacityById,
           pybind11::arg("streetId"),
           pybind11::arg("factor"),
           R"doc(Change street capacity by id.

      Args:
        streetId (int): Street identifier.
        factor (float): Multiplicative capacity factor.)doc")
      .def("changeStreetCapacityByName",
           &dsf::mobility::RoadNetwork::changeStreetCapacityByName,
           pybind11::arg("name"),
           pybind11::arg("factor"),
           R"doc(Change street capacity by name.

      Args:
        name (str): Street name.
        factor (float): Multiplicative capacity factor.)doc")
      .def("addCoil",
           &dsf::mobility::RoadNetwork::addCoil,
           pybind11::arg("streetId"),
           pybind11::arg("name") = std::string(),
           R"doc(Add a coil sensor to a street.

      Args:
        streetId (int): Street identifier.
        name (str): Optional name for the coil.)doc")
      .def("shortestPath",
           &dsf::mobility::RoadNetwork::shortestPath,
           pybind11::arg("sourceId"),
           pybind11::arg("targetId"),
           R"doc(Compute the shortest path between two node ids.

      Args:
        sourceId (int): Source node id.
        targetId (int): Target node id.

      Returns:
        list[int]: Sequence of node ids representing the shortest path.)doc")
      .def("computeBetweennessCentralities",
           &dsf::mobility::RoadNetwork::computeBetweennessCentralities,
           R"doc(Compute betweenness centralities for nodes in the network.)doc")
      .def("computeEdgeBetweennessCentralities",
           &dsf::mobility::RoadNetwork::computeEdgeBetweennessCentralities,
           R"doc(Compute betweenness centralities for edges in the network.)doc")
      .def("computeEdgeKBetweennessCentralities",
           &dsf::mobility::RoadNetwork::computeEdgeKBetweennessCentralities,
           pybind11::arg("k"),
           R"doc(Compute k-edge betweenness centralities.

      Args:
        k (int): Parameter controlling the computation.)doc")
      .def(
          "nodeBetweennessCentralities",
          [](const dsf::mobility::RoadNetwork& self) {
            std::unordered_map<dsf::Id, std::optional<double>> result;
            for (auto const& [nodeId, pNode] : self.nodes()) {
              result[nodeId] =
                  pNode->template getAttribute<double>("betweennessCentrality");
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
              result[edgeId] =
                  pEdge->template getAttribute<double>("betweennessCentrality");
            }
            return result;
          },
          "Get the betweenness centrality values for all edges.\n\n"
          "Returns:\n"
          "    dict[int, float | None]: A dictionary mapping edge id to its "
          "betweenness centrality value (None if not computed).")
      .def(
          "exportCSV",
          [](const dsf::mobility::RoadNetwork& self, const std::string& outputDir) {
            self.exportCSV(outputDir);
          },
          pybind11::arg("outputDir"),
          R"doc(Export network data to CSV files in the given directory.

      Args:
        outputDir (str): Destination directory path.)doc");

  pybind11::class_<dsf::PathCollection>(mobility, "PathCollection")
      .def(pybind11::init<>(), "Create an empty PathCollection")
      .def(
          "__getitem__",
          [](const dsf::PathCollection& self, dsf::Id key) {
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
          [](dsf::PathCollection& self, dsf::Id key, std::vector<dsf::Id> value) {
            self[key] = value;
          },
          pybind11::arg("key"),
          pybind11::arg("value"),
          "Set the next hops for a given node id")
      .def(
          "__contains__",
          [](const dsf::PathCollection& self, dsf::Id key) {
            return self.find(key) != self.end();
          },
          pybind11::arg("key"),
          "Check if a node id exists in the collection")
      .def(
          "__len__",
          [](const dsf::PathCollection& self) { return self.size(); },
          "Get the number of nodes in the collection")
      .def(
          "keys",
          [](const dsf::PathCollection& self) {
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
          [](const dsf::PathCollection& self) {
            pybind11::dict items;
            for (const auto& [key, value] : self) {
              items[pybind11::int_(key)] = pybind11::cast(value);
            }
            return items;
          },
          "Get all items (node id, next hops) in the collection")
      .def("explode",
           &dsf::PathCollection::explode,
           pybind11::arg("sourceId"),
           pybind11::arg("targetId"),
           R"doc(Build a path from a source node to a target node.

Args:
    sourceId (int): Starting node id.
    targetId (int): Ending node id.

Returns:
    PathCollection: A path collection containing the computed route.)doc");

  pybind11::class_<dsf::mobility::Itinerary>(mobility, "Itinerary")
      .def(pybind11::init<dsf::Id, dsf::Id>(),
           pybind11::arg("id"),
           pybind11::arg("destination"),
           R"doc(Create an itinerary with an id and destination node.

Args:
    id (int): Itinerary identifier.
    destination (int): Destination node id.

Returns:
    Itinerary: A new itinerary instance.)doc")
      .def("setPath",
           &dsf::mobility::Itinerary::setPath,
           pybind11::arg("path"),
           R"doc(Set the path used by this itinerary.

Args:
    path (Sequence[int]): Sequence of node ids representing the route.

Returns:
    None)doc")
      .def("id",
           &dsf::mobility::Itinerary::id,
           R"doc(Get the itinerary identifier.

Returns:
    int: The itinerary id.)doc")
      .def("destination",
           &dsf::mobility::Itinerary::destination,
           R"doc(Get the destination node id.

Returns:
    int: The destination node id.)doc");
  // .def("path", &dsf::mobility::Itinerary::path, pybind11::return_value_policy::reference_internal);

  pybind11::class_<dsf::mobility::FirstOrderDynamics>(mobility, "Dynamics")
      //     // Constructors are not directly exposed due to the template nature and complexity.
      //     // Users should use derived classes like FirstOrderDynamics.
      .def(pybind11::init([](dsf::mobility::RoadNetwork& graph,
                             bool useCache,
                             std::optional<unsigned int> seed) {
             return std::unique_ptr<dsf::mobility::FirstOrderDynamics>(
                 new dsf::mobility::FirstOrderDynamics(std::move(graph), useCache, seed));
           }),
           pybind11::arg("graph"),
           pybind11::arg("useCache") = false,
           pybind11::arg("seed") = std::nullopt,
           R"doc(Create a dynamics model over a road network.

Args:
    graph (RoadNetwork): Road network used by the simulation.
    useCache (bool, optional): Enable internal caching. Defaults to False.
    seed (int | None, optional): Random seed for reproducibility.

Returns:
    Dynamics: A new dynamics instance.)doc")
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
           R"doc(Set the simulation name.

    Args:
      name (str): New name for the dynamics object.

    Returns:
      None)doc")
      .def("setInitTime",
           &dsf::mobility::FirstOrderDynamics::setInitTime,
           pybind11::arg("timeEpoch"),
           R"doc(Set the simulation start time from an epoch timestamp.

    Args:
      timeEpoch (int): Epoch timestamp in seconds.

    Returns:
      None)doc")
      .def(
          "setInitTime",
          [](dsf::mobility::FirstOrderDynamics& self, pybind11::object datetime_obj) {
            auto const epoch_seconds =
                pybind11::cast<double>(datetime_obj.attr("timestamp")());
            self.setInitTime(static_cast<std::time_t>(epoch_seconds));
          },
          pybind11::arg("datetime"),
          R"doc(Set the simulation start time from a Python datetime-like object.

      Args:
        datetime (datetime): Object exposing a timestamp() method.

      Returns:
        None)doc")
      .def("setConcurrency",
           &dsf::mobility::FirstOrderDynamics::setConcurrency,
           pybind11::arg("concurrency"),
           R"doc(Set the concurrency level used by the simulation.

    Args:
      concurrency (int): Number of concurrent workers or threads.

    Returns:
      None)doc")
      .def("connectDataBase",
           &dsf::mobility::FirstOrderDynamics::connectDataBase,
           pybind11::arg("dbPath"),
           pybind11::arg("queries") =
               "PRAGMA busy_timeout = 5000;PRAGMA journal_mode = WAL;PRAGMA "
               "synchronous=NORMAL;PRAGMA temp_store=MEMORY;PRAGMA cache_size=-20000;",
           R"doc(Connect the simulation to a SQLite database.

Args:
    dbPath (str): Path to the SQLite database file.
    queries (str, optional): Initialization SQL statements.

Returns:
    None)doc")
      .def("setForcePriorities",
           &dsf::mobility::FirstOrderDynamics::setForcePriorities,
           pybind11::arg("forcePriorities"),
           R"doc(Enable or disable force-based route priorities.

    Args:
      forcePriorities (bool): Whether to force priority handling.

    Returns:
      None)doc")
      .def(
          "setDataUpdatePeriod",
          [](dsf::mobility::FirstOrderDynamics& self, int dataUpdatePeriod) {
            self.setDataUpdatePeriod(static_cast<dsf::Delay>(dataUpdatePeriod));
          },
          pybind11::arg("dataUpdatePeriod"),
          R"doc(Set the interval between data updates.

Args:
    dataUpdatePeriod (int): Update period in simulation time units.

Returns:
    None)doc")
      .def("setMeanTravelDistance",
           &dsf::mobility::FirstOrderDynamics::setMeanTravelDistance,
           pybind11::arg("meanDistance"),
           R"doc(Set the target mean travel distance.

Args:
    meanDistance (float): Mean travel distance.

Returns:
    None)doc")
      .def(
          "setMeanTravelTime",
          [](dsf::mobility::FirstOrderDynamics& self, uint64_t meanTravelTime) {
            self.setMeanTravelTime(static_cast<std::time_t>(meanTravelTime));
          },
          pybind11::arg("meanTravelTime"),
          R"doc(Set the target mean travel time.

Args:
    meanTravelTime (int): Mean travel time in seconds.

Returns:
    None)doc")
      .def("setErrorProbability",
           &dsf::mobility::FirstOrderDynamics::setErrorProbability,
           pybind11::arg("errorProbability"),
           R"doc(Set the probability of injecting simulation errors.

    Args:
      errorProbability (float): Probability in the range [0, 1].

    Returns:
      None)doc")
      .def("killStagnantAgents",
           &dsf::mobility::FirstOrderDynamics::killStagnantAgents,
           pybind11::arg("timeToleranceFactor") = 3.,
           R"doc(Enable or configure removal of stagnant agents.

    Args:
      timeToleranceFactor (float, optional): Multiplier used to detect stagnation.

    Returns:
      None)doc")
      .def(
          "setDestinationNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             const std::vector<dsf::Id>& destinationNodes) {
            self.setDestinationNodes(destinationNodes);
          },
          pybind11::arg("destinationNodes"),
          R"doc(Set the destination nodes using a list of node ids.

Args:
    destinationNodes (Sequence[int]): Destination node ids.

Returns:
    None)doc")
      .def(
          "setOriginNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             const std::unordered_map<dsf::Id, double>& originNodes) {
            self.setOriginNodes(originNodes);
          },
          pybind11::arg("originNodes") = std::unordered_map<dsf::Id, double>(),
          R"doc(Set weighted origin nodes from a mapping of node ids to weights.

Args:
    originNodes (Mapping[int, float], optional): Origin nodes and weights.

Returns:
    None)doc")
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
          R"doc(Set origin nodes from a numpy array of node ids.

Args:
    originNodes (array[int]): Node ids to use as origins.

Returns:
    None)doc")
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
          R"doc(Set destination nodes from a numpy array of node ids.

Args:
    destinationNodes (array[int]): Node ids to use as destinations.

Returns:
    None)doc")
      .def(
          "setDestinationNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             const std::unordered_map<dsf::Id, double>& destinationNodes) {
            self.setDestinationNodes(destinationNodes);
          },
          pybind11::arg("destinationNodes"),
          R"doc(Set weighted destination nodes from a mapping of node ids to weights.

      Args:
        destinationNodes (Mapping[int, float]): Destination nodes and weights.

      Returns:
        None)doc")
      .def("setODs",
           &dsf::mobility::FirstOrderDynamics::setODs,
           pybind11::arg("ods"),
           R"doc(Set origin-destination pairs for the simulation.

    Args:
      ods (object): Origin-destination data structure expected by the C++ API.

    Returns:
      None)doc")
      .def(
          "importODsFromCSV",
          [](dsf::mobility::FirstOrderDynamics& self,
             const std::string& fileName,
             char separator = ';') { self.importODsFromCSV(fileName, separator); },
          pybind11::arg("fileName"),
          pybind11::arg("separator") = ';',
          "Import origin-destination pairs from a CSV file.\n\n"
          "Supports two CSV formats:\n"
          "1. RANDOM_ODS: columns (node_id, type, weight) where type is 'O' (origin) or "
          "'D' (destination)\n"
          "2. ODS: columns (origin_id, destination_id, weight) for explicit OD pairs\n\n"
          "Args:\n"
          "    fileName (str): Path to the CSV file\n"
          "    separator (str): CSV delimiter character (default is ';')")
      .def("initTurnCounts",
           &dsf::mobility::FirstOrderDynamics::initTurnCounts,
           R"doc(Initialize turn count tracking data structures.

    Returns:
      None)doc")
      .def("updatePaths",
           &dsf::mobility::FirstOrderDynamics::updatePaths,
           pybind11::arg("throw_on_empty") = true,
           R"doc(Recompute routing paths for the current network state.

    Args:
      throw_on_empty (bool, optional): Raise if no path can be found.

    Returns:
      None)doc")
      .def("addAgentsUniformly",
           &dsf::mobility::FirstOrderDynamics::addAgentsUniformly,
           pybind11::arg("nAgents"),
           pybind11::arg("itineraryId") = std::nullopt,
           R"doc(Add agents uniformly across the configured origins.

    Args:
      nAgents (int): Number of agents to add.
      itineraryId (int | None, optional): Itinerary to assign.

    Returns:
      None)doc")
      .def(
          "addAgents",
          [](dsf::mobility::FirstOrderDynamics& self,
             std::size_t nAgents,
             dsf::mobility::AgentInsertionMethod insertionMethod) {
            self.addAgents(nAgents, insertionMethod);
          },
          pybind11::arg("nAgents"),
          pybind11::arg("insertionMethod"),
          R"doc(Add agents using the requested insertion method.

      Args:
        nAgents (int): Number of agents to add.
        insertionMethod (AgentInsertionMethod): Insertion policy.

      Returns:
        None)doc")
      .def("evolve",
           &dsf::mobility::FirstOrderDynamics::evolve,
           pybind11::arg("reinsert_agents") = false,
           R"doc(Advance the simulation by one time step.

      Args:
        reinsert_agents (bool, optional): Reinsert agents after movement.

      Returns:
        None)doc")
      .def("optimizeTrafficLights",
           &dsf::mobility::FirstOrderDynamics::optimizeTrafficLights,
           pybind11::arg("optimizationType") = dsf::TrafficLightOptimization::DOUBLE_TAIL,
           pybind11::arg("logFile") = "",
           pybind11::arg("threshold") = 0.,
           pybind11::arg("ratio") = 1.3,
           R"doc(Optimize traffic light timing for the current network.

Args:
    optimizationType (TrafficLightOptimization, optional): Optimization mode.
    logFile (str, optional): Optional log file path.
    threshold (float, optional): Threshold used by the optimizer.
    ratio (float, optional): Optimization ratio parameter.

Returns:
    None)doc")
      .def(
          "graph",
          [](dsf::mobility::FirstOrderDynamics& self) -> dsf::mobility::RoadNetwork& {
            return self.graph();
          },
          pybind11::return_value_policy::reference_internal,
          R"doc(Get the underlying road network.

      Returns:
        RoadNetwork: Reference to the simulation graph.)doc")
      .def("nAgents",
           &dsf::mobility::FirstOrderDynamics::nAgents,
           R"doc(Get the current number of agents in the simulation.

    Returns:
      int: Number of active agents.)doc")
      .def("time",
           &dsf::mobility::FirstOrderDynamics::time,
           R"doc(Get the current simulation time.

    Returns:
      int: Simulation time step or timestamp.)doc")
      .def("time_step",
           &dsf::mobility::FirstOrderDynamics::time_step,
           R"doc(Get the simulation time step increment.

    Returns:
      int: Time step size.)doc")
      .def("datetime",
           &dsf::mobility::FirstOrderDynamics::strDateTime,
           R"doc(Get the current simulation time formatted as a string.

    Returns:
      str: Human-readable date/time string.)doc")
      .def("meanTravelTime",
           &dsf::mobility::FirstOrderDynamics::meanTravelTime,
           pybind11::arg("clearData") = false,
           R"doc(Compute the mean travel time of agents.

Args:
    clearData (bool, optional): Clear accumulated statistics after reading.

Returns:
    float: Mean travel time.)doc")
      .def("meanTravelDistance",
           &dsf::mobility::FirstOrderDynamics::meanTravelDistance,
           pybind11::arg("clearData") = false,
           R"doc(Compute the mean travel distance of agents.

Args:
    clearData (bool, optional): Clear accumulated statistics after reading.

Returns:
    float: Mean travel distance.)doc")
      .def("meanTravelSpeed",
           &dsf::mobility::FirstOrderDynamics::meanTravelSpeed,
           pybind11::arg("clearData") = false,
           R"doc(Compute the mean travel speed of agents.

Args:
    clearData (bool, optional): Clear accumulated statistics after reading.

Returns:
    float: Mean travel speed.)doc")
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
          R"doc(Get the turn counts grouped by origin and destination.

      Returns:
        dict[int, dict[int, int]]: Nested mapping of turn counts.)doc")
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
          R"doc(Get normalized turn counts grouped by origin and destination.

Returns:
    dict[int, dict[int, float]]: Nested mapping of normalized counts.)doc")
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
          R"doc(Get the number of origin events per node.

      Args:
        reset (bool, optional): Reset counters after reading.

      Returns:
        dict[int, int]: Mapping of node id to origin count.)doc")
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
          R"doc(Get the number of destination events per node.

Args:
    reset (bool, optional): Reset counters after reading.

Returns:
    dict[int, int]: Mapping of node id to destination count.)doc")
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
          R"doc(Print a summary of the current simulation state.

      Returns:
        None)doc");

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
           R"doc(Create a trajectory collection from a file.

      Args:
        fileName (str): Input file path.
        column_mapping (dict[str, str], optional): Column name mapping.
        separator (str, optional): CSV delimiter.
        bbox (Sequence[float], optional): Bounding box coordinates.

      Returns:
        TrajectoryCollection: Loaded trajectory collection.)doc")
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
           R"doc(Filter trajectories using cluster, speed, and duration constraints.

      Args:
        cluster_radius_km (float): Maximum cluster radius in kilometers.
        max_speed_kph (float, optional): Maximum allowed speed.
        min_points_per_trajectory (int, optional): Minimum number of points.
        min_duration_min (int | None, optional): Minimum duration in minutes.

      Returns:
        TrajectoryCollection: Filtered collection.)doc")
      .def("to_csv",
           &dsf::mdt::TrajectoryCollection::to_csv,
           pybind11::arg("fileName"),
           pybind11::arg("sep") = ';',
           R"doc(Export the trajectory collection to a CSV file.

      Args:
        fileName (str): Output file path.
        sep (str, optional): Field separator.

      Returns:
        None)doc")
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
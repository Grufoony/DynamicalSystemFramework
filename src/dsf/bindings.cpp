#include "dsf.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/unordered_set.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

namespace nb = nanobind;

NB_MODULE(dsf_cpp, m) {
  m.doc() = "Python bindings for the DSF library";
  m.attr("__version__") = dsf::version();

  // Create mobility submodule
  auto mobility = m.def_submodule("mobility",
                                  "Bindings for mobility-related classes and functions, "
                                  "under the dsf::mobility C++ namespace.");
  auto mdt = m.def_submodule("mdt",
                             "Bindings for movement data tools (MDT) related classes and "
                             "functions, under the dsf::mdt C++ namespace.");

  auto logging =
      m.def_submodule("logging",
                      "Bindings for logging-related classes and functions, under the "
                      "spdlog C++ namespace.");

  // Bind AgentInsertionMethod enum
  nb::enum_<dsf::mobility::AgentInsertionMethod>(mobility, "AgentInsertionMethod")
      .value("ODS", dsf::mobility::AgentInsertionMethod::ODS)
      .value("RANDOM", dsf::mobility::AgentInsertionMethod::RANDOM)
      .value("RANDOM_WEIGHTED_ORIGIN",
             dsf::mobility::AgentInsertionMethod::RANDOM_WEIGHTED_ORIGIN)
      .value("RANDOM_ODS", dsf::mobility::AgentInsertionMethod::RANDOM_ODS)
      .value("CONDITIONAL_RANDOM_ODS",
             dsf::mobility::AgentInsertionMethod::CONDITIONAL_RANDOM_ODS)
      .value("UNIFORM", dsf::mobility::AgentInsertionMethod::UNIFORM)
      .export_values();

  // Bind TrafficLightOptimization enum
  nb::enum_<dsf::TrafficLightOptimization>(mobility, "TrafficLightOptimization")
      .value("SINGLE_TAIL", dsf::TrafficLightOptimization::SINGLE_TAIL)
      .value("DOUBLE_TAIL", dsf::TrafficLightOptimization::DOUBLE_TAIL)
      .export_values();

  // Bind SpeedFunction enum
  nb::enum_<dsf::SpeedFunction>(mobility, "SpeedFunction")
      .value("CUSTOM", dsf::SpeedFunction::CUSTOM)
      .value("LINEAR", dsf::SpeedFunction::LINEAR)
      .export_values();

  // Bind Direction enum
  nb::enum_<dsf::Direction>(mobility, "Direction")
      .value("RIGHT", dsf::Direction::RIGHT)
      .value("STRAIGHT", dsf::Direction::STRAIGHT)
      .value("LEFT", dsf::Direction::LEFT)
      .value("UTURN", dsf::Direction::UTURN)
      .value("RIGHTANDSTRAIGHT", dsf::Direction::RIGHTANDSTRAIGHT)
      .value("LEFTANDSTRAIGHT", dsf::Direction::LEFTANDSTRAIGHT)
      .value("ANY", dsf::Direction::ANY)
      .export_values();

  // Bind RoadStatus enum
  nb::enum_<dsf::mobility::RoadStatus>(mobility, "RoadStatus")
      .value("OPEN", dsf::mobility::RoadStatus::OPEN)
      .value("CLOSED", dsf::mobility::RoadStatus::CLOSED)
      .export_values();

  // Bind spdlog log level enum
  nb::enum_<spdlog::level::level_enum>(logging, "LogLevel")
      .value("TRACE", spdlog::level::trace)
      .value("DEBUG", spdlog::level::debug)
      .value("INFO", spdlog::level::info)
      .value("WARN", spdlog::level::warn)
      .value("ERROR", spdlog::level::err)
      .value("CRITICAL", spdlog::level::critical)
      .value("OFF", spdlog::level::off)
      .export_values();

  logging.def(
      "set_level",
      [](spdlog::level::level_enum level) { spdlog::set_level(level); },
      nb::arg("level"),
      "Set the global log level");

  logging.def(
      "to_file",
      [](std::string const& fileName) {
        try {
          auto file_logger = spdlog::basic_logger_mt("dsf_file_logger", fileName);
          spdlog::set_default_logger(file_logger);
          spdlog::info("Logging to file: {}", fileName);
        } catch (const spdlog::spdlog_ex& ex) {
          spdlog::error("Log initialization failed: {}", ex.what());
        }
      },
      nb::arg("fileName"),
      "Configure the global logger to write to a file");

  logging.def(
      "info",
      [](std::string const& message) { spdlog::info("{}", message); },
      nb::arg("message"),
      "Log an info message");

  logging.def(
      "warn",
      [](std::string const& message) { spdlog::warn("{}", message); },
      nb::arg("message"),
      "Log a warning message");

  logging.def(
      "error",
      [](std::string const& message) { spdlog::error("{}", message); },
      nb::arg("message"),
      "Log an error message");

  logging.def(
      "debug",
      [](std::string const& message) { spdlog::debug("{}", message); },
      nb::arg("message"),
      "Log a debug message");

  // Bind Street class to mobility submodule
  auto street =
      nb::class_<dsf::mobility::Street>(mobility, "Street")
          .def(
              "__repr__",
              [](const dsf::mobility::Street& s) { return std::format("{}", s); },
              R"doc(Return a string representation of the Street object.)doc")
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
        float: Maximum speed in m/s.)doc")
          .def("capacity",
               &dsf::mobility::Street::capacity,
               R"doc(Get the capacity of this street.

    Returns:
        int: Capacity as the number of vehicles.)doc")
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
          .def("nExitingAgents",
               &dsf::mobility::Street::nExitingAgents,
               nb::arg("direction") = dsf::Direction::ANY,
               nb::arg("normalizeOnNLanes") = false,
               R"doc(Get the number of agents exiting this street in the current step.
    Args:
        direction (Direction, optional): Filter exiting agents by direction. Default is ANY.
        normalizeOnNLanes (bool, optional): If true, normalize the count by the number

    Returns:
        int: Number of exiting agents.)doc")
          .def(
              "density",
              [&](dsf::mobility::Street const& self, bool normalized) {
                if (normalized) {
                  return self.density<true>();
                }
                return self.density<false>();
              },
              nb::arg("normalized") = false,
              R"doc(Get the current density of agents on this street.
    Args:
        normalized (bool, optional): If true, return density normalized by capacity.

    Returns:
        float: Density value.)doc")
          .def("estimatedTravelTime",
               &dsf::mobility::Street::estimatedTravelTime,
               "Get estimated travel time for this street using the active estimator.")
          .def("attributes",
               &dsf::mobility::Street::attributes,
               R"doc(Get the attribute dictionary for this street.

    Returns:
        dict: Mapping of attribute names to values.)doc");

  // Expose coil counter helpers and road-level static setter
  street
      .def_static("setMeanVehicleLength",
                  &dsf::mobility::Road::setMeanVehicleLength,
                  nb::arg("meanVehicleLength"),
                  R"doc(Set the global mean vehicle length in metres.)doc")
      .def("counts",
           &dsf::mobility::Street::counts,
           R"doc(Get the counts of the coil (if enabled).

    Returns:
        int: Number of counts recorded by the coil, or 0 if none.)doc")
      .def("resetCounter",
           &dsf::mobility::Street::resetCounter,
           R"doc(Reset the coil counter on this street.)doc");

  // Bind RoadJunction class to mobility submodule
  nb::class_<dsf::mobility::RoadJunction>(mobility, "RoadJunction")
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
        int: Capacity value.)doc")
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

  // Bind TrafficLightPhase and TrafficLight classes.
  nb::class_<dsf::mobility::TrafficLightPhase>(mobility, "TrafficLightPhase")
      .def(nb::init<dsf::Delay>(),
           nb::arg("duration"),
           R"doc(Create a TrafficLightPhase(duration).)doc")
      .def(nb::init<dsf::Delay,
                    std::unordered_map<dsf::Id, std::unordered_set<dsf::Direction>>>(),
           nb::arg("duration"),
           nb::arg("greenSet"),
           R"doc(Create a TrafficLightPhase(duration, greenSet).)doc")
      .def(
          "__repr__",
          [](const dsf::mobility::TrafficLightPhase& p) { return std::format("{}", p); },
          R"doc(Return a string representation of the TrafficLightPhase object.)doc")
      .def("addGreen",
           nb::overload_cast<dsf::Id, dsf::Direction>(
               &dsf::mobility::TrafficLightPhase::addGreen),
           nb::arg("streetId"),
           nb::arg("direction"))
      .def("addGreen",
           nb::overload_cast<dsf::Id>(&dsf::mobility::TrafficLightPhase::addGreen),
           nb::arg("streetId"),
           R"doc(Add a street to the phase's green set.

        Args:
          streetId (int): Identifier of the street to grant green.

        Returns:
          None)doc")
      .def("containsStreet",
           &dsf::mobility::TrafficLightPhase::containsStreet,
           R"doc(Check whether the phase controls the given street.

        Args:
          streetId (int): Street identifier to query.

        Returns:
          bool: True if the street is part of the phase's green set.)doc")
      .def("containsGreen",
           &dsf::mobility::TrafficLightPhase::containsGreen,
           R"doc(Check whether the phase grants green for a given direction on a street.

        Args:
          streetId (int): Street identifier.
          direction (Direction): Direction to query.

        Returns:
          bool: True if the direction on the street is green in this phase.)doc")
      .def("duration",
           &dsf::mobility::TrafficLightPhase::duration,
           R"doc(Get the duration of this traffic light phase.

        Returns:
          Delay: Phase duration in simulation time units.)doc")
      .def("setDuration",
           &dsf::mobility::TrafficLightPhase::setDuration,
           R"doc(Set the duration of this traffic light phase.

        Args:
          duration (Delay): New phase duration.

        Returns:
          None)doc")
      .def("greenSet",
           &dsf::mobility::TrafficLightPhase::greenSet,
           nb::rv_policy::reference_internal,
           R"doc(Return the internal green set for this phase.

        Returns:
          Mapping[int, set[Direction]]: Internal mapping of street id to allowed directions (reference).)doc");

  nb::class_<dsf::mobility::Intersection, dsf::mobility::RoadJunction>(mobility,
                                                                       "Intersection")
      .def(nb::init<dsf::Id>(), nb::arg("id"))
      .def(nb::init<dsf::Id, dsf::geometry::Point>(), nb::arg("id"), nb::arg("point"))
      .def(
          "__repr__",
          [](const dsf::mobility::Intersection& i) { return std::format("{}", i); },
          R"doc(Return a string representation of the Intersection object.)doc");

  nb::class_<dsf::mobility::TrafficLight, dsf::mobility::RoadJunction>(mobility,
                                                                       "TrafficLight")
      .def(nb::init<dsf::Id>(), nb::arg("id"))
      .def(nb::init<dsf::Id, dsf::geometry::Point>(), nb::arg("id"), nb::arg("point"))
      .def(nb::init<dsf::mobility::RoadJunction const&>(), nb::arg("node"))
      .def(
          "__repr__",
          [](const dsf::mobility::TrafficLight& tl) { return std::format("{}", tl); },
          R"doc(Return a string representation of the TrafficLight object.)doc")
      .def("setAllowFreeTurns",
           &dsf::mobility::TrafficLight::setAllowFreeTurns,
           nb::arg("allow"),
           R"doc(Enable or disable free (uncontrolled) turning movements at this junction.

      Args:
          allow (bool): True to allow free turns, False to enforce phases.

      Returns:
          None)doc")
      .def("addPhase",
           &dsf::mobility::TrafficLight::addPhase,
           nb::arg("phase"),
           R"doc(Add a traffic light phase to the end of the phase list.

      Args:
          phase (TrafficLightPhase): Phase to append.

      Returns:
          None)doc")
      .def("setPhases",
           &dsf::mobility::TrafficLight::setPhases,
           nb::arg("phases"),
           R"doc(Replace the current phases with the provided sequence.

      Args:
          phases (Sequence[TrafficLightPhase]): New ordered list of phases.

      Returns:
          None)doc")
      .def(
          "clearPhases",
          &dsf::mobility::TrafficLight::clearPhases,
          R"doc(Remove all configured phases, leaving the traffic light with no phases.)doc")
      .def("snapshot",
           &dsf::mobility::TrafficLight::snapshot,
           R"doc(Capture the current traffic light state for later restoration.)doc")
      .def(
          "restore",
          &dsf::mobility::TrafficLight::restore,
          R"doc(Restore the traffic light state previously captured by `snapshot()`.)doc")
      .def("reset",
           &dsf::mobility::TrafficLight::reset,
           R"doc(Reset the traffic light to its default configuration and timing.)doc")
      .def("isGreen",
           &dsf::mobility::TrafficLight::isGreen,
           nb::arg("streetId"),
           nb::arg("direction"),
           R"doc(Check whether a given street and direction currently has green.

      Args:
          streetId (int): Street identifier.
          direction (Direction): Direction to query.

      Returns:
          bool: True if green, False otherwise.)doc")
      .def(
          "cycleTime",
          &dsf::mobility::TrafficLight::cycleTime,
          R"doc(Get the total cycle time of the traffic light (sum of all phase durations).

      Returns:
          Delay: Cycle time in simulation units.)doc")
      .def("meanGreenTime",
           &dsf::mobility::TrafficLight::meanGreenTime,
           nb::arg("priorityStreets"),
           R"doc(Compute mean green time for a set of priority streets.

      Args:
          priorityStreets (Sequence[int]): Streets to consider as priority.

      Returns:
          double: Mean green duration for the provided streets.)doc")
      .def("isDefault",
           &dsf::mobility::TrafficLight::isDefault,
           R"doc(Check whether the traffic light is currently using its default phases.

      Returns:
          bool: True if default phases are active.)doc")
      .def("advanceBy",
           &dsf::mobility::TrafficLight::advanceBy,
           nb::arg("offset"),
           R"doc(Advance the internal phase clock by the given offset.

      Args:
          offset (Delay): Amount to advance the clock by.

      Returns:
          None)doc")
      .def("counter",
           &dsf::mobility::TrafficLight::counter,
           R"doc(Get the internal event counter used for optimizations or statistics.

      Returns:
          size_t: Counter value.)doc")
      .def("currentPhaseIndex",
           &dsf::mobility::TrafficLight::currentPhaseIndex,
           R"doc(Get the index of the currently active phase.

      Returns:
          int: Current phase index (0-based).)doc")
      .def("phases",
           &dsf::mobility::TrafficLight::phases,
           nb::rv_policy::reference_internal,
           R"doc(Return the list of configured phases (reference).

      Returns:
          Sequence[TrafficLightPhase]: Reference to phase list.)doc")
      .def(
          "defaultPhases",
          &dsf::mobility::TrafficLight::defaultPhases,
          nb::rv_policy::reference_internal,
          R"doc(Return the default phase configuration for this traffic light (reference).

      Returns:
          Sequence[TrafficLightPhase]: Reference to default phase list.)doc")
      .def("phase",
           &dsf::mobility::TrafficLight::phase,
           nb::arg("index"),
           nb::rv_policy::reference_internal,
           R"doc(Get a reference to the phase at the given index.

      Args:
          index (int): Index of the phase to retrieve.

      Returns:
          TrafficLightPhase: Reference to the requested phase.)doc");

  // Bind Measurement to main module
  nb::class_<dsf::Measurement<double>>(m, "Measurement")
      .def(nb::init<double, double, std::size_t>(),
           nb::arg("mean"),
           nb::arg("std"),
           nb::arg("n"),
           R"doc(Create a Measurement object.

    Args:
      mean (float): Sample mean.
      std (float): Sample standard deviation.
      n (int): Number of samples.

    Returns:
      Measurement: Initialized measurement instance.)doc")
      .def_rw("mean",
              &dsf::Measurement<double>::mean,
              R"doc(Mean value of the measurement.

                Type:
                  float)doc")
      .def_rw("std",
              &dsf::Measurement<double>::std,
              R"doc(Standard deviation of the measurement.

          Type:
            float)doc")
      .def_rw("n",
              &dsf::Measurement<double>::n,
              R"doc(Number of samples used to compute the measurement.

          Type:
            int)doc")
      .def_rw("is_valid",
              &dsf::Measurement<double>::is_valid,
              R"doc(Flag indicating whether the measurement is valid.

          Type:
            bool)doc");

  // Bind mobility-related classes to mobility submodule
  nb::class_<dsf::mobility::RoadNetwork>(mobility, "RoadNetwork")
      .def(nb::init<>(),
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
      .def(
          "addNode",
          [](dsf::mobility::RoadNetwork& self, dsf::Id id) {
            self.addNode(dsf::mobility::RoadJunction(id));
          },
          nb::arg("id"),
          R"doc(Add a node with the given id to the network.)doc")
      .def(
          "addStreet",
          [](dsf::mobility::RoadNetwork& self,
             dsf::Id id,
             dsf::Id source,
             dsf::Id target,
             double length,
             double maxSpeed,
             int nLanes,
             std::string name) {
            self.addStreet(dsf::mobility::Street(
                id, std::make_pair(source, target), length, maxSpeed, nLanes, name));
          },
          nb::arg("id"),
          nb::arg("source"),
          nb::arg("target"),
          nb::arg("length"),
          nb::arg("maxSpeed"),
          nb::arg("nLanes") = 1,
          nb::arg("name") = std::string(),
          R"doc(Add a street to the network using simple parameters.
       
       Args:
         id (int): Street identifier.
         source (int): Source node identifier.
         target (int): Target node identifier.
         length (float): Street length.
         maxSpeed (float): Maximum speed on the street.
         nLanes (int): Number of lanes.
         name (str): Street name.

       Returns:
         None)doc")
      .def(
          "node",
          [](dsf::mobility::RoadNetwork& self, dsf::Id nodeId)
              -> dsf::mobility::RoadJunction& { return self.node(nodeId); },
          nb::arg("nodeId"),
          nb::rv_policy::reference_internal,
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
          nb::arg("edgeId"),
          nb::rv_policy::reference_internal,
          R"doc(Get a reference to the edge with the given id.

      Args:
        edgeId (int): Edge identifier.

      Returns:
        Street: Reference to the requested street.)doc")
      .def("capacity",
           &dsf::mobility::RoadNetwork::capacity,
           R"doc(Get the capacity value for the network or a specific resource.

      Returns:
        int: Capacity.)doc")
      .def("adjustNodeCapacities",
           &dsf::mobility::RoadNetwork::adjustNodeCapacities,
           R"doc(Adjust capacities of nodes according to provided factors.

      Args:
        (See C++ API) Adjusts node capacities in-place.)doc")
      .def("autoInitTrafficLights",
           &dsf::mobility::RoadNetwork::autoInitTrafficLights,
           nb::arg("mainRoadPercentage") = 0.6,
           nb::arg("defaultCycleDuration") = 90,
           R"doc(Auto-initialize traffic lights based on network heuristics.

      Args:
        mainRoadPercentage (float, optional): Fraction used to identify main roads. Default 0.6
        defaultCycleDuration (int, optional): Default cycle duration in ticks. Default 90)doc")
      .def(
          "autoMapStreetLanes",
          &dsf::mobility::RoadNetwork::autoMapStreetLanes,
          R"doc(Auto-map lanes for streets using heuristics based on geometry and attributes.)doc")
      .def("setEdgeWeight",
           &dsf::mobility::RoadNetwork::setEdgeWeight,
           nb::arg("weight"),
           nb::arg("threshold") = std::nullopt,
           R"doc(Set edge weights for routing and analysis.

      Args:
        weight (string): Weight type (e.g., 'length', 'traveltime').
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
          nb::arg("fileName"),
          R"doc(Import edges from a CSV file.

      Args:
        fileName (str): Path to the CSV file.)doc")
      .def(
          "importEdges",
          [](dsf::mobility::RoadNetwork& self,
             std::string const& fileName,
             char const separator) { self.importEdges(fileName, separator); },
          nb::arg("fileName"),
          nb::arg("separator"),
          R"doc(Import edges from a CSV file with a custom separator.

      Args:
        fileName (str): Path to the CSV file.
        separator (str): Field separator character.)doc")
      .def(
          "importEdges",
          [](dsf::mobility::RoadNetwork& self,
             std::string const& fileName,
             bool const bCreateInverse) { self.importEdges(fileName, bCreateInverse); },
          nb::arg("fileName"),
          nb::arg("bCreateInverse"),
          R"doc(Import edges from a CSV file and optionally create inverse edges.

      Args:
        fileName (str): Path to the CSV file.
        bCreateInverse (bool): Create inverse edges when true.)doc")
      .def(
          "importNodeProperties",
          [](dsf::mobility::RoadNetwork& self,
             std::string const& fileName,
             char const separator) { self.importNodeProperties(fileName, separator); },
          nb::arg("fileName"),
          nb::arg("separator") = ';',
          R"doc(Import node properties from a CSV file.

      Args:
        fileName (str): Path to the CSV file.
        separator (str): Field separator character.)doc")
      .def("importTrafficLights",
           &dsf::mobility::RoadNetwork::importTrafficLights,
           nb::arg("fileName"),
           R"doc(Import traffic light configurations from a file.

      Args:
        fileName (str): Path to the configuration file.)doc")
      .def("exportTrafficLights",
           &dsf::mobility::RoadNetwork::exportTrafficLights,
           nb::arg("fileName"),
           R"doc(Export traffic light configurations to a file.

      Args:
        fileName (str): Path to the configuration file.)doc")
      .def(
          "makeRoundabout",
          [](dsf::mobility::RoadNetwork& self, dsf::Id id) -> void {
            self.makeRoundabout(id);
          },
          nb::arg("id"),
          R"doc(Convert the node with the given id into a roundabout.

      Args:
        id (int): Node id to convert.)doc")
      .def(
          "makeTrafficLight",
          [](dsf::mobility::RoadNetwork& self, dsf::Id id) -> void {
            self.makeTrafficLight(id);
          },
          nb::arg("id"),
          R"doc(Create a traffic light at the given node id.

      Args:
        id (int): Node id.)doc")
      .def("setStreetStatusById",
           &dsf::mobility::RoadNetwork::setStreetStatusById,
           nb::arg("streetId"),
           nb::arg("status"),
           R"doc(Set the status of a street by id.

      Args:
        streetId (int): Street identifier.
        status (RoadStatus): New status to set.)doc")
      .def("setStreetStatusByName",
           &dsf::mobility::RoadNetwork::setStreetStatusByName,
           nb::arg("name"),
           nb::arg("status"),
           R"doc(Set the status of a street by name.

      Args:
        name (str): Street name.
        status (RoadStatus): New status to set.)doc")
      .def("changeStreetNLanesById",
           &dsf::mobility::RoadNetwork::changeStreetNLanesById,
           nb::arg("streetId"),
           nb::arg("nLanes"),
           nb::arg("speedFactor") = std::nullopt,
           R"doc(Change the number of lanes for a street by id.

      Args:
        streetId (int): Street identifier.
        nLanes (int): New number of lanes.
        speedFactor (float | None): Optional speed factor.)doc")
      .def("changeStreetNLanesByName",
           &dsf::mobility::RoadNetwork::changeStreetNLanesByName,
           nb::arg("name"),
           nb::arg("nLanes"),
           nb::arg("speedFactor") = std::nullopt,
           R"doc(Change the number of lanes for a street by name.

      Args:
        name (str): Street name.
        nLanes (int): New number of lanes.
        speedFactor (float | None): Optional speed factor.)doc")
      .def("changeStreetCapacityById",
           &dsf::mobility::RoadNetwork::changeStreetCapacityById,
           nb::arg("streetId"),
           nb::arg("factor"),
           R"doc(Change street capacity by id.

      Args:
        streetId (int): Street identifier.
        factor (float): Multiplicative capacity factor.)doc")
      .def("changeStreetCapacityByName",
           &dsf::mobility::RoadNetwork::changeStreetCapacityByName,
           nb::arg("name"),
           nb::arg("factor"),
           R"doc(Change street capacity by name.

      Args:
        name (str): Street name.
        factor (float): Multiplicative capacity factor.)doc")
      .def("addCoil",
           &dsf::mobility::RoadNetwork::addCoil,
           nb::arg("streetId"),
           nb::arg("name") = std::string(),
           R"doc(Add a coil sensor to a street.

      Args:
        streetId (int): Street identifier.
        name (str): Optional name for the coil.)doc")
      .def("shortestPath",
           &dsf::mobility::RoadNetwork::shortestPath,
           nb::arg("sourceId"),
           nb::arg("targetId"),
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
           nb::arg("k"),
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
          "allPathsTo",
          [](const dsf::mobility::RoadNetwork& self, dsf::Id targetId) {
            return self.allPathsTo(targetId);
          },
          nb::arg("targetId"),
          R"doc(Compute all paths leading to a target node.
      Args:
        targetId (int): Target node id.
      Returns:
        PathCollection: A collection of paths leading to the target node.)doc")
      .def(
          "allEdgePathsTo",
          [](const dsf::mobility::RoadNetwork& self, dsf::Id targetId) {
            return self.allEdgePathsTo(targetId);
          },
          nb::arg("targetId"),
          R"doc(Compute all edge paths leading to a target edge.
      Args:
        targetId (int): Target edge id.
      Returns:
        PathCollection: A collection of edge paths leading to the target edge.)doc")
      .def(
          "exportCSV",
          [](const dsf::mobility::RoadNetwork& self, const std::string& outputDir) {
            self.exportCSV(outputDir);
          },
          nb::arg("outputDir"),
          R"doc(Export network data to CSV files in the given directory.

      Args:
        outputDir (str): Destination directory path.)doc");

  nb::class_<dsf::PathCollection>(mobility, "PathCollection")
      .def(nb::init<>(), "Create an empty PathCollection")
      .def(
          "__getitem__",
          [](const dsf::PathCollection& self, dsf::Id key) {
            auto it = self.find(key);
            if (it == self.end()) {
              throw nb::key_error("Key not found");
            }
            return it->second;
          },
          nb::arg("key"),
          "Get the next hops for a given node id")
      .def(
          "__setitem__",
          [](dsf::PathCollection& self, dsf::Id key, std::vector<dsf::Id> value) {
            self[key] = value;
          },
          nb::arg("key"),
          nb::arg("value"),
          "Set the next hops for a given node id")
      .def(
          "__contains__",
          [](const dsf::PathCollection& self, dsf::Id key) {
            return self.find(key) != self.end();
          },
          nb::arg("key"),
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
            nb::dict items;
            for (const auto& [key, value] : self) {
              items[nb::cast(key)] = nb::cast(value);
            }
            return items;
          },
          "Get all items (node id, next hops) in the collection")
      .def("explode",
           &dsf::PathCollection::explode,
           nb::arg("sourceId"),
           nb::arg("targetId"),
           R"doc(Build a path from a source node to a target node.

Args:
    sourceId (int): Starting node id.
    targetId (int): Ending node id.

Returns:
    PathCollection: A path collection containing the computed route.)doc");

  nb::class_<dsf::mobility::Itinerary>(mobility, "Itinerary")
      .def(nb::init<dsf::Id, dsf::Id>(),
           nb::arg("id"),
           nb::arg("destination"),
           R"doc(Create an itinerary with an id and destination node.

Args:
    id (int): Itinerary identifier.
    destination (int): Destination node id.

Returns:
    Itinerary: A new itinerary instance.)doc")
      .def("setPath",
           &dsf::mobility::Itinerary::setPath,
           nb::arg("path"),
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

  nb::class_<dsf::mobility::FirstOrderDynamics>(mobility, "Dynamics")
      .def(nb::init<dsf::mobility::RoadNetwork&&, bool, std::optional<unsigned int>>(),
           nb::arg("graph"),
           nb::arg("useCache") = false,
           nb::arg("seed") = std::nullopt,
           R"doc(Create a FirstOrderDynamics instance.)doc")
      .def("setSeed",
           &dsf::mobility::FirstOrderDynamics::setSeed,
           nb::arg("seed"),
           R"doc(Set the seed value for the random number generator.

      Args:
          seed (int): The seed value for the random number generator.

      Returns:
          None)doc")
      .def(
          "setSpeedFunction",
          [](dsf::mobility::FirstOrderDynamics& self,
             dsf::SpeedFunction speedFunction,
             nb::object arg) {
            switch (speedFunction) {
              case dsf::SpeedFunction::LINEAR:
                self.setSpeedFunction(dsf::SpeedFunction::LINEAR, nb::cast<double>(arg));
                break;
              case dsf::SpeedFunction::CUSTOM: {
                auto* func_ptr = reinterpret_cast<double (*)(double, double)>(
                    nb::cast<uintptr_t>(arg));
                self.setSpeedFunction(
                    dsf::SpeedFunction::CUSTOM,
                    [func_ptr](dsf::mobility::Street const& street) -> double {
                      return func_ptr(street.maxSpeed(), street.density<true>());
                    });
                break;
              }
              default:
                throw std::invalid_argument("Invalid speed function type");
            }
          },
          nb::arg("speedFunction"),
          nb::arg("arg"),
          R"doc(Set the speed function for agents.

      Args:
          speedFunction (SpeedFunction): The speed function type (LINEAR or CUSTOM)
          arg: For LINEAR, a float alpha in [0., 1.). For CUSTOM, an integer address (uintptr_t) of a C function with signature double(double max_speed, double density).)doc")
      .def("setConcurrency",
           &dsf::mobility::FirstOrderDynamics::setConcurrency,
           nb::arg("concurrency"),
           R"doc(Set the concurrency level used by the simulation.

      Args:
        concurrency (int): Number of concurrent workers or threads.

      Returns:
        None)doc")
      .def("setForcePriorities",
           &dsf::mobility::FirstOrderDynamics::setForcePriorities,
           nb::arg("forcePriorities"),
           R"doc(Enable or disable force-based route priorities.

      Args:
        forcePriorities (bool): Whether to force priority handling.

      Returns:
        None)doc")
      .def("setUpdatePathsThrowOnEmpty",
           &dsf::mobility::FirstOrderDynamics::setUpdatePathsThrowOnEmpty,
           nb::arg("throwOnEmpty"),
           R"doc(Enable or disable throwing an exception when paths are empty.

      Args:
        throwOnEmpty (bool): Whether to throw an exception when paths are empty.

      Returns:
        None)doc")
      .def(
          "setDataUpdatePeriod",
          [](dsf::mobility::FirstOrderDynamics& self, int dataUpdatePeriod) {
            self.setDataUpdatePeriod(static_cast<dsf::Delay>(dataUpdatePeriod));
          },
          nb::arg("dataUpdatePeriod"),
          R"doc(Set the interval between data updates.

      Args:
          dataUpdatePeriod (int): Update period in simulation time units.

      Returns:
          None)doc")
      .def("setMeanTravelDistance",
           &dsf::mobility::FirstOrderDynamics::setMeanTravelDistance,
           nb::arg("meanDistance"),
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
          nb::arg("meanTravelTime"),
          R"doc(Set the target mean travel time.

      Args:
          meanTravelTime (int): Mean travel time in seconds.

      Returns:
          None)doc")
      .def("setErrorProbability",
           &dsf::mobility::FirstOrderDynamics::setErrorProbability,
           nb::arg("errorProbability"),
           R"doc(Set the probability of injecting simulation errors.

      Args:
        errorProbability (float): Probability in the range [0, 1].

      Returns:
        None)doc")
      .def("killStagnantAgents",
           &dsf::mobility::FirstOrderDynamics::killStagnantAgents,
           nb::arg("timeToleranceFactor") = 3.,
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
          nb::arg("destinationNodes"),
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
          nb::arg("originNodes") = std::unordered_map<dsf::Id, double>(),
          R"doc(Set weighted origin nodes from a mapping of node ids to weights.

Args:
    originNodes (Mapping[int, float], optional): Origin nodes and weights.

Returns:
    None)doc")
      .def(
          "setOriginNodes",
          [](dsf::mobility::FirstOrderDynamics& self, nb::ndarray<dsf::Id> originNodes) {
            auto* ptr = static_cast<dsf::Id*>(originNodes.data());
            std::unordered_map<dsf::Id, double> nodeWeights;
            for (size_t i = 0; i < originNodes.size(); ++i) {
              nodeWeights[ptr[i]] = 1.0;
            }
            self.setOriginNodes(nodeWeights);
          },
          nb::arg("originNodes"),
          R"doc(Set origin nodes from a numpy array of node ids.

Args:
    originNodes (array[int]): Node ids to use as origins.

Returns:
    None)doc")
      .def(
          "setDestinationNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             nb::ndarray<dsf::Id> destinationNodes) {
            auto* ptr = static_cast<dsf::Id*>(destinationNodes.data());
            std::vector<dsf::Id> nodes(ptr, ptr + destinationNodes.size());
            self.setDestinationNodes(nodes);
          },
          nb::arg("destinationNodes"),
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
          nb::arg("destinationNodes"),
          R"doc(Set weighted destination nodes from a mapping of node ids to weights.

      Args:
        destinationNodes (Mapping[int, float]): Destination nodes and weights.

      Returns:
        None)doc")
      .def("setODs",
           &dsf::mobility::FirstOrderDynamics::setODs,
           nb::arg("ods"),
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
          nb::arg("fileName"),
          nb::arg("separator") = ';',
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
           R"doc(Recompute routing paths for the current network state.

    Args:
      throw_on_empty (bool, optional): Raise if no path can be found.

    Returns:
      None)doc")
      .def(
          "addItinerary",
          [](dsf::mobility::FirstOrderDynamics& self, dsf::Id id, dsf::Id destination) {
            self.addItinerary(id, destination);
          },
          nb::arg("id"),
          nb::arg("destination"),
          R"doc(Add an itinerary by id and destination.

    Args:
      id (int): Itinerary id.
      destination (int): Destination node id.

    Returns:
      None)doc")
      .def(
          "itineraries",
          [](const dsf::mobility::FirstOrderDynamics& self) {
            nb::dict py_result;
            for (const auto& [id, pItin] : self.itineraries()) {
              py_result[nb::cast(id)] = nb::cast(pItin);
            }
            return py_result;
          },
          R"doc(Get the itineraries mapping as a dict[id, Itinerary].)doc")
      .def("addAgentsUniformly",
           &dsf::mobility::FirstOrderDynamics::addAgentsUniformly,
           nb::arg("nAgents"),
           nb::arg("itineraryId") = std::nullopt,
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
          nb::arg("nAgents"),
          nb::arg("insertionMethod"),
          R"doc(Add agents using the requested insertion method.

      Args:
        nAgents (int): Number of agents to add.
        insertionMethod (AgentInsertionMethod): Insertion policy.

      Returns:
        None)doc")
      .def(
          "evolve",
          [](dsf::mobility::FirstOrderDynamics& self) { self.evolve(); },
          R"doc(Advance the simulation by one time step.

      Returns:
        None)doc")
      .def("optimizeTrafficLights",
           &dsf::mobility::FirstOrderDynamics::optimizeTrafficLights,
           nb::arg("optimizationType") = dsf::TrafficLightOptimization::DOUBLE_TAIL,
           nb::arg("logFile") = "",
           nb::arg("threshold") = 0.,
           nb::arg("ratio") = 1.3,
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
          nb::rv_policy::reference_internal,
          R"doc(Get the underlying road network.

      Returns:
        RoadNetwork: Reference to the simulation graph.)doc")
      .def("nAgents",
           &dsf::mobility::FirstOrderDynamics::nAgents,
           R"doc(Get the current number of agents in the simulation.

    Returns:
      int: Number of active agents.)doc")
      .def("meanTravelTime",
           &dsf::mobility::FirstOrderDynamics::meanTravelTime,
           nb::arg("clearData") = false,
           R"doc(Compute the mean travel time of agents.

        Args:
            clearData (bool, optional): Clear accumulated statistics after reading.

        Returns:
            float: Mean travel time.)doc")
      .def("meanTravelDistance",
           &dsf::mobility::FirstOrderDynamics::meanTravelDistance,
           nb::arg("clearData") = false,
           R"doc(Compute the mean travel distance of agents.

        Args:
            clearData (bool, optional): Clear accumulated statistics after reading.

        Returns:
            float: Mean travel distance.)doc")
      .def("meanTravelSpeed",
           &dsf::mobility::FirstOrderDynamics::meanTravelSpeed,
           nb::arg("clearData") = false,
           R"doc(Compute the mean travel speed of agents.

        Args:
            clearData (bool, optional): Clear accumulated statistics after reading.

        Returns:
            float: Mean travel speed.)doc")
      .def(
          "turnCounts",
          [](const dsf::mobility::FirstOrderDynamics& self) {
            nb::dict py_result;
            for (const auto& [from_id, inner_map] : self.turnCounts()) {
              nb::dict py_inner;
              for (const auto& [to_id, count] : inner_map) {
                py_inner[nb::cast(to_id)] = nb::cast(count);
              }
              py_result[nb::cast(from_id)] = py_inner;
            }
            return py_result;
          },
          R"doc(Get the turn counts grouped by origin and destination.

      Returns:
        dict[int, dict[int, int]]: Nested mapping of turn counts.)doc")
      .def(
          "normalizedTurnCounts",
          [](const dsf::mobility::FirstOrderDynamics& self) {
            nb::dict py_result;
            for (const auto& [from_id, inner_map] : self.normalizedTurnCounts()) {
              nb::dict py_inner;
              for (const auto& [to_id, count] : inner_map) {
                py_inner[nb::cast(to_id)] = nb::cast(count);
              }
              py_result[nb::cast(from_id)] = py_inner;
            }
            return py_result;
          },
          R"doc(Get normalized turn counts grouped by origin and destination.

      Returns:
          dict[int, dict[int, float]]: Nested mapping of normalized counts.)doc")
      .def(
          "originCounts",
          [](dsf::mobility::FirstOrderDynamics& self, bool reset) {
            nb::dict py_result;
            for (const auto& [node_id, count] : self.originCounts(reset)) {
              py_result[nb::cast(node_id)] = nb::cast(count);
            }
            return py_result;
          },
          nb::arg("reset") = true,
          R"doc(Get the number of origin events per node.

      Args:
        reset (bool, optional): Reset counters after reading.

      Returns:
        dict[int, int]: Mapping of node id to origin count.)doc")
      .def(
          "destinationCounts",
          [](dsf::mobility::FirstOrderDynamics& self, bool reset) {
            nb::dict py_result;
            for (const auto& [node_id, count] : self.destinationCounts(reset)) {
              py_result[nb::cast(node_id)] = nb::cast(count);
            }
            return py_result;
          },
          nb::arg("reset") = true,
          R"doc(Get the number of destination events per node.

      Args:
          reset (bool, optional): Reset counters after reading.

      Returns:
          dict[int, int]: Mapping of node id to destination count.)doc")
      .def(
          "summary",
          [](dsf::mobility::FirstOrderDynamics& self) {
            self.summary();  // Uses default std::cout
          },
          R"doc(Print a summary of the current simulation state.

      Returns:
        None)doc");

  // Bind TrafficSimulator class to mobility submodule
  nb::class_<dsf::mobility::TrafficSimulator>(mobility, "TrafficSimulator")
      .def(nb::init<>())
      .def(nb::init<std::string const&>(),
           nb::arg("jsonConfigFile"),
           R"doc(Create a TrafficSimulator instance from a JSON configuration file.
      Args:
          jsonConfigFile (str): Path to the JSON configuration file.

      Returns:
          TrafficSimulator: A new instance of the traffic simulator initialized with the provided configuration.)doc")
      .def(
          "connectDataBase",
          nb::overload_cast<std::string_view, std::string_view>(
              &dsf::mobility::TrafficSimulator::connectDataBase),
          nb::arg("dbPath"),
          nb::arg("queries") =
              "PRAGMA busy_timeout = 5000;PRAGMA journal_mode = WAL;PRAGMA "
              "synchronous=NORMAL;PRAGMA temp_store=MEMORY;PRAGMA cache_size=-20000;",
          R"doc(Connect the simulator to a SQLite database and configure connection pragmas.

    Args:
      dbPath (str): Path to the SQLite database file.
      queries (str, optional): Initialization PRAGMA statements to run on connect.

    Returns:
      None)doc")
      .def("setOutputPrefix",
           &dsf::mobility::TrafficSimulator::setOutputPrefix,
           nb::arg("prefix"),
           R"doc(Set the output prefix for saved data files.

      Args:
        prefix (str): The prefix for the output files.

      Returns:
        None)doc")
      .def(
          "importRoadNetwork",
          [](dsf::mobility::TrafficSimulator& self,
             const std::string& edgesFile,
             const std::string& nodePropertiesFile) {
            self.importRoadNetwork(edgesFile, nodePropertiesFile);
          },
          nb::arg("edgesFile"),
          nb::arg("nodePropertiesFile") = std::string(),
          R"doc(Import the road network from edge and optional node property files.

      Args:
        edgesFile (str): Path to CSV file containing edges definitions.
        nodePropertiesFile (str, optional): Path to CSV file with node attributes.

      Returns:
        None)doc")
      .def("updatePaths",
           &dsf::mobility::TrafficSimulator::updatePaths,
           nb::arg("deltaT") = 0,
           nb::arg("throwOnEmpty") = true,
           R"doc(Recompute routing paths for the current network state.
      Args:
      deltaT (int, optional): Time interval between path updates.
      throwOnEmpty (bool, optional): Raise exception for empty paths.

      Returns:
      None)doc")
      .def("saveData",
           &dsf::mobility::TrafficSimulator::saveData,
           nb::arg("savingInterval"),
           nb::arg("saveAverageStats") = false,
           nb::arg("saveStreetData") = false,
           nb::arg("saveTravelData") = false,
           nb::arg("saveAgentData") = false,
           nb::arg("saveTurnCountsData") = false,
           R"doc(Save simulation data according to the requested options.

    Args:
      savingInterval (int): Interval (seconds) at which to save data.
      saveAverageStats (bool): Save aggregated statistics when True.
      saveStreetData (bool): Save per-street data when True.
      saveTravelData (bool): Save travel statistics when True.
      saveAgentData (bool): Save per-agent traces when True.
      saveTurnCountsData (bool): Save turn counts when True.

    Returns:
      None)doc")
      .def("setName",
           &dsf::mobility::TrafficSimulator::setName,
           nb::arg("name"),
           R"doc(Set a name for the traffic simulation instance.)

      Args:
        name (str): New name for the traffic simulation instance.

      Returns:
        None)doc")
      .def(
          "setTimeFrame",
          [](dsf::mobility::TrafficSimulator& self,
             std::uint64_t initTime,
             nb::object endTime) {
            if (endTime.is_none()) {
              self.setTimeFrame(static_cast<std::time_t>(initTime));
            } else {
              auto end = static_cast<std::time_t>(nb::cast<std::uint64_t>(endTime));
              self.setTimeFrame(static_cast<std::time_t>(initTime),
                                std::optional<std::time_t>(end));
            }
          },
          nb::arg("initTime"),
          nb::arg("endTime") = nb::none(),
          R"doc(Set the simulation time frame.

      Args:
        initTime (int): Start time in epoch seconds.
        endTime (int | None): Optional end time in epoch seconds.

      Returns:
        None)doc")
      .def("setAgentInsertionMethod",
           &dsf::mobility::TrafficSimulator::setAgentInsertionMethod,
           nb::arg("insertionMethod"),
           R"doc(Set how new agents are inserted into the simulation.

    Args:
      insertionMethod (AgentInsertionMethod): Policy controlling agent insertion.

    Returns:
      None)doc")
      .def(
          "run",
          [](dsf::mobility::TrafficSimulator& self,
             std::vector<std::size_t> nAgentsPerTimeStep,
             std::optional<std::time_t> deltaT,
             double const percentageRandomAgents) {
            self.run(nAgentsPerTimeStep, deltaT, percentageRandomAgents);
          },
          nb::arg("nAgentsPerTimeStep"),
          nb::arg("deltaT") = std::nullopt,
          nb::arg("percentageRandomAgents") = 0.0,
          R"doc(Run the simulation in default mode.

      Args:
          nAgentsPerTimeStep: Number of agents to insert at each scheduled insertion step.
          deltaT: Optional interval in seconds between agent insertions. If omitted,
                  the interval is inferred from the configured start/end times and the
                  length of nAgentsPerTimeStep.
          percentageRandomAgents: Optional percentage of random agents to insert at each step.
      )doc")
      .def(
          "run",
          [](dsf::mobility::TrafficSimulator& self,
             std::size_t nInitialAgents,
             std::time_t agentInsertionDeltaT,
             std::time_t checkDeltaT,
             std::size_t agentIncrement) {
            self.run(nInitialAgents, agentInsertionDeltaT, checkDeltaT, agentIncrement);
          },
          nb::arg("nInitialAgents"),
          nb::arg("agentInsertionDeltaT"),
          nb::arg("checkDeltaT"),
          nb::arg("agentIncrement") = 1,
          R"doc(Run the simulation in slow-charge mode.

        Gradually ramps up the agent population: every checkDeltaT seconds the current
        agent count is compared against the target; if it has fallen below, the target
        is raised by agentIncrement and a fresh batch is injected every
        agentInsertionDeltaT seconds.

        Args:
          nInitialAgents:        Starting target number of agents in the network.
          agentInsertionDeltaT:  Interval in seconds between agent insertion attempts.
          checkDeltaT:           Interval in seconds between occupancy checks.
          agentIncrement:        How many agents to add to the target at each check (default 1).
        )doc")
      .def(
          "database",
          [](dsf::mobility::TrafficSimulator& self) { return self.database(); },
          nb::rv_policy::reference,
          R"doc(Get a reference to the simulator's connected database object.

    Returns:
      Database: Reference to the database connection object.)doc")
      .def(
          "dynamics",
          [](dsf::mobility::TrafficSimulator& self) { return self.dynamics(); },
          nb::rv_policy::reference,
          R"doc(Get a reference to the simulator's dynamics engine.

    Returns:
      Dynamics: Reference to the configured dynamics instance.)doc")
      .def("id",
           &dsf::mobility::TrafficSimulator::id,
           R"doc(Get the simulator instance identifier.

Returns:
    int: Simulator id.)doc")
      .def("initTime",
           &dsf::mobility::TrafficSimulator::initTime,
           R"doc(Get the configured simulation start time as epoch seconds.

Returns:
    int: Start time in epoch seconds.)doc")
      .def("strInitTime",
           &dsf::mobility::TrafficSimulator::strInitTime,
           R"doc(Get the configured simulation start time as a human-readable string.

Returns:
    str: Formatted start time.)doc")
      .def("endTime",
           &dsf::mobility::TrafficSimulator::endTime,
           R"doc(Get the configured simulation end time as epoch seconds.

Returns:
    int: End time in epoch seconds.)doc")
      .def("strEndTime",
           &dsf::mobility::TrafficSimulator::strEndTime,
           R"doc(Get the configured simulation end time as a human-readable string.

Returns:
    str: Formatted end time.)doc")
      .def("name",
           &dsf::mobility::TrafficSimulator::name,
           R"doc(Get the user-provided name for this simulation instance.

Returns:
    str: Simulation name.)doc")
      .def("safeName",
           &dsf::mobility::TrafficSimulator::safeName,
           R"doc(Get a filesystem-safe version of the simulation name.

Returns:
    str: Safe name suitable for file prefixes.)doc");

  // Bind TrajectoryCollection class to mdt submodule
  nb::class_<dsf::mdt::TrajectoryCollection>(mdt, "TrajectoryCollection")
      .def(nb::init<std::string const&,
                    std::unordered_map<std::string, std::string> const&,
                    char const,
                    std::array<double, 4> const&>(),
           nb::arg("fileName"),
           nb::arg("column_mapping") = std::unordered_map<std::string, std::string>{},
           nb::arg("separator") = ';',
           nb::arg("bbox") = std::array<double, 4>{},
           R"doc(Create a trajectory collection from a file.

      Args:
        fileName (str): Input file path.
        column_mapping (dict[str, str], optional): Column name mapping.
        separator (str, optional): CSV delimiter.
        bbox (Sequence[float], optional): Bounding box coordinates.

      Returns:
        TrajectoryCollection: Loaded trajectory collection.)doc")
      .def(
          "__init__",
          [](dsf::mdt::TrajectoryCollection* self, nb::object df) {
            nb::object columns = df.attr("columns");
            nb::object arr_obj = df.attr("to_numpy")();

            nb::ndarray<nb::ro> arr = nb::cast<nb::ndarray<nb::ro>>(arr_obj);
            if (arr.ndim() != 2) {
              throw std::runtime_error(
                  "TrajectoryCollection constructor expects a 2D numpy array from "
                  "df.to_numpy()");
            }
            std::size_t n_rows = arr.shape(0);
            std::size_t n_cols = arr.shape(1);

            // Collect column names
            std::vector<std::string> colnames;
            for (auto item : columns) {
              colnames.push_back(nb::cast<std::string>(nb::str(item)));
            }

            std::unordered_map<std::string,
                               std::variant<std::vector<dsf::Id>,
                                            std::vector<std::time_t>,
                                            std::vector<double>>>
                dataframe;
            dataframe.reserve(n_cols);

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
                  nb::object cell = arr_obj[nb::make_tuple(i, 0)];
                  std::get<std::vector<dsf::Id>>(dataframe.at("uid"))
                      .push_back(static_cast<dsf::Id>(nb::cast<double>(cell)));
                }
              } else if (colname == "timestamp") {
                for (std::size_t i = 0; i < n_rows; ++i) {
                  nb::object cell = arr_obj[nb::make_tuple(i, 1)];
                  std::get<std::vector<std::time_t>>(dataframe.at("timestamp"))
                      .push_back(static_cast<std::time_t>(nb::cast<double>(cell)));
                }
              } else if (colname == "lat") {
                for (std::size_t i = 0; i < n_rows; ++i) {
                  nb::object cell = arr_obj[nb::make_tuple(i, 2)];
                  std::get<std::vector<double>>(dataframe.at("lat"))
                      .push_back(nb::cast<double>(cell));
                }
              } else if (colname == "lon") {
                for (std::size_t i = 0; i < n_rows; ++i) {
                  nb::object cell = arr_obj[nb::make_tuple(i, 3)];
                  std::get<std::vector<double>>(dataframe.at("lon"))
                      .push_back(nb::cast<double>(cell));
                }
              }
            }

            new (self) dsf::mdt::TrajectoryCollection(std::move(dataframe));
          },
          nb::arg("df"),
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
           nb::arg("cluster_radius_km"),
           nb::arg("max_speed_kph") = 150.0,
           nb::arg("min_points_per_trajectory") = 2,
           nb::arg("min_duration_min") = nb::none(),
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
           nb::arg("fileName"),
           nb::arg("sep") = ';',
           R"doc(Export the trajectory collection to a CSV file.

      Args:
        fileName (str): Output file path.
        sep (str, optional): Field separator.

      Returns:
        None)doc")
      .def(
          "to_pandas",
          [](const dsf::mdt::TrajectoryCollection& self) {
            nb::module_ pd = nb::module_::import_("pandas");
            nb::dict data_dict;

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

            data_dict["uid"] = uids;
            data_dict["trajectory_id"] = trajectoryIds;
            data_dict["lon"] = lons;
            data_dict["lat"] = lats;
            data_dict["timestamp_in"] = timestamps_in;
            data_dict["timestamp_out"] = timestamps_out;

            return pd.attr("DataFrame")(data_dict);
          },
          "Convert the TrajectoryCollection to a pandas DataFrame.\n\nReturns:\n\tpandas."
          "DataFrame: DataFrame containing the trajectory data with columns 'uid', "
          "'trajectory_id', 'lon', 'lat', 'timestamp_in', and 'timestamp_out'.")
      .def(
          "to_polars",
          [](const dsf::mdt::TrajectoryCollection& self) {
            nb::module_ pl = nb::module_::import_("polars");
            nb::dict data_dict;

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

            data_dict["uid"] = uids;
            data_dict["trajectory_id"] = trajectoryIds;
            data_dict["lon"] = lons;
            data_dict["lat"] = lats;
            data_dict["timestamp_in"] = timestamps_in;
            data_dict["timestamp_out"] = timestamps_out;

            return pl.attr("DataFrame")(data_dict);
          },
          R"doc(Convert the TrajectoryCollection to a polars DataFrame.\n\nReturns:\n\tpolars."
          "DataFrame: DataFrame containing the trajectory data with columns 'uid', "
          "'trajectory_id', 'lon', 'lat', 'timestamp_in', and 'timestamp_out'.)doc");
}

#include "Itinerary.hpp"

#include <fstream>
#include <stdexcept>

namespace dsf::mobility {
  Itinerary::Itinerary(Id id, Id destination) : m_id{id}, m_destination{destination} {}

  void Itinerary::load(const std::string& fileName) {
    // Open binary file
    std::ifstream inFile{fileName, std::ios::binary};
    if (!inFile.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for reading.");
    }
    // Load the m_path variable from the file
    inFile.read(reinterpret_cast<char*>(&m_destination), sizeof(Id));
    size_t mapSize;
    inFile.read(reinterpret_cast<char*>(&mapSize), sizeof(size_t));
    m_nodePath.clear();
    m_nodePath.reserve(mapSize);
    for (size_t i = 0; i < mapSize; ++i) {
      Id key;
      inFile.read(reinterpret_cast<char*>(&key), sizeof(Id));
      size_t vecSize;
      inFile.read(reinterpret_cast<char*>(&vecSize), sizeof(size_t));
      std::vector<Id> vec(vecSize);
      inFile.read(reinterpret_cast<char*>(vec.data()), vecSize * sizeof(Id));
      m_nodePath.emplace(key, std::move(vec));
    }

    inFile.close();
  }

  void Itinerary::save(const std::string& fileName) const {
    // Open binary file
    std::ofstream outFile{fileName, std::ios::binary};
    if (!outFile.is_open()) {
      throw std::runtime_error("Error opening file \"" + fileName + "\" for writing.");
    }
    outFile.write(reinterpret_cast<const char*>(&m_destination), sizeof(Id));
    // Save the m_nodePath and m_edgePath variables in the file
    size_t mapSize = m_nodePath.size();
    outFile.write(reinterpret_cast<const char*>(&mapSize), sizeof(size_t));
    for (auto const& [key, vec] : m_nodePath) {
      outFile.write(reinterpret_cast<const char*>(&key), sizeof(Id));
      size_t vecSize = vec.size();
      outFile.write(reinterpret_cast<const char*>(&vecSize), sizeof(size_t));
      outFile.write(reinterpret_cast<const char*>(vec.data()), vecSize * sizeof(Id));
    }

    outFile.close();
  }

};  // namespace dsf::mobility

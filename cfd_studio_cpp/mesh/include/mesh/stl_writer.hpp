#pragma once

#include <string>

#include "mesh/mesh.hpp"

namespace cfd::mesh {

// Writes a binary STL file (not ASCII -- see stl_writer.cpp for why: read
// performance at the triangle counts this is actually used at). Throws
// std::runtime_error if the file can't be opened or the write fails. The
// counterpart to read_stl (mesh/stl_reader.hpp) -- cfd_mesh previously had
// readers for all four formats but no writer at all.
void write_stl(const Mesh& mesh, const std::string& path);

} // namespace cfd::mesh

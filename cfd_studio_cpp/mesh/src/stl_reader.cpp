#include "mesh/stl_reader.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

namespace cfd::mesh {

namespace {

Mesh read_stl_binary(std::ifstream& f, std::uint32_t triangle_count) {
    Mesh mesh;
    mesh.vertices.reserve(static_cast<std::size_t>(triangle_count) * 3);
    mesh.triangles.reserve(triangle_count);

    // Read the whole triangle block in one call instead of 3 small
    // f.read()s per triangle (normal/verts/attr): with hundreds of
    // thousands of triangles that's millions of tiny std::ifstream::read()
    // calls, each paying real per-call iostream overhead (sentry
    // construction, streambuf virtual dispatch). One big read plus
    // in-memory pointer walking is both far fewer stream calls and,
    // incidentally, simpler.
    constexpr std::size_t kRecordSize = 12 + 36 + 2; // normal(3 floats) + verts(9 floats) + attribute byte count
    std::vector<char> buffer(static_cast<std::size_t>(triangle_count) * kRecordSize);
    f.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    if (!f) throw std::runtime_error("read_stl: truncated binary STL file");

    const char* p = buffer.data();
    for (std::uint32_t t = 0; t < triangle_count; ++t) {
        p += 12; // normal -- unused, mesh normals are recomputed from vertices elsewhere
        float verts[9];
        std::memcpy(verts, p, sizeof(verts));
        p += 36 + 2; // verts, then skip the attribute byte count

        auto base = static_cast<std::uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back({verts[0], verts[1], verts[2]});
        mesh.vertices.push_back({verts[3], verts[4], verts[5]});
        mesh.vertices.push_back({verts[6], verts[7], verts[8]});
        mesh.triangles.push_back({base, base + 1, base + 2});
    }
    return mesh;
}

Mesh read_stl_ascii(std::ifstream& f) {
    Mesh mesh;
    std::string token;
    double x, y, z;
    std::vector<Vec3> pending_verts;

    while (f >> token) {
        if (token == "vertex") {
            f >> x >> y >> z;
            pending_verts.push_back({x, y, z});
            if (pending_verts.size() == 3) {
                auto base = static_cast<std::uint32_t>(mesh.vertices.size());
                mesh.vertices.push_back(pending_verts[0]);
                mesh.vertices.push_back(pending_verts[1]);
                mesh.vertices.push_back(pending_verts[2]);
                mesh.triangles.push_back({base, base + 1, base + 2});
                pending_verts.clear();
            }
        }
        // All other tokens (solid, facet, normal, outer, loop, endloop,
        // endfacet, endsolid, and the solid name) are skipped -- we only
        // need geometry, not the STL name/annotation fields.
    }
    return mesh;
}

} // namespace

Mesh read_stl(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("read_stl: cannot open file: " + path);

    // Binary STL: 80-byte header, 4-byte little-endian triangle count, then
    // 50 bytes per triangle. Detect by checking whether the file's actual
    // size matches that exact formula -- more robust than checking for a
    // "solid" prefix, since some binary STL files (non-conformant but real)
    // still start with the bytes "solid" in their 80-byte header.
    f.seekg(0, std::ios::end);
    auto file_size = static_cast<std::uint64_t>(f.tellg());
    f.seekg(0, std::ios::beg);

    if (file_size >= 84) {
        char header[80];
        f.read(header, 80);
        std::uint32_t triangle_count = 0;
        f.read(reinterpret_cast<char*>(&triangle_count), sizeof(triangle_count));
        std::uint64_t expected_size = 84ULL + 50ULL * triangle_count;
        if (expected_size == file_size) {
            return read_stl_binary(f, triangle_count);
        }
    }

    f.clear();
    f.seekg(0, std::ios::beg);
    Mesh mesh = read_stl_ascii(f);
    if (mesh.triangles.empty()) {
        throw std::runtime_error("read_stl: no triangles parsed (not a valid ASCII or binary STL): " + path);
    }
    return mesh;
}

} // namespace cfd::mesh

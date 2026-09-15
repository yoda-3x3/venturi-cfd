#include "mesh/stl_writer.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>

namespace cfd::mesh {

void write_stl(const Mesh& mesh, const std::string& path) {
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("write_stl: cannot open file for writing: " + path);

    // Binary format (80-byte header, uint32 triangle count, then 50 bytes
    // per triangle -- matches read_stl_binary()'s expectations exactly),
    // not ASCII: for a mesh with hundreds of thousands of triangles (the
    // cut-cell-refined domain mesh this is chiefly used for), ASCII's
    // formatted-float text was several times larger on disk and, more
    // importantly, read back via millions of individual operator>> token
    // extractions -- a 10+ second parse that made opening the results
    // viewer report "not responding". Binary read/write is both a couple
    // of large block transfers plus fast in-memory pointer walking.
    char header[80] = {};
    std::memcpy(header, "cfd_mesh binary STL", 20);
    f.write(header, sizeof(header));

    auto triangleCount = static_cast<std::uint32_t>(mesh.triangles.size());
    f.write(reinterpret_cast<const char*>(&triangleCount), sizeof(triangleCount));

    for (const auto& tri : mesh.triangles) {
        const Vec3& a = mesh.vertices[tri[0]];
        const Vec3& b = mesh.vertices[tri[1]];
        const Vec3& c = mesh.vertices[tri[2]];
        Vec3 n = normalize(cross(b - a, c - a));

        float normal[3] = {static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z)};
        float verts[9] = {
            static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z),
            static_cast<float>(b.x), static_cast<float>(b.y), static_cast<float>(b.z),
            static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z),
        };
        std::uint16_t attr = 0;
        f.write(reinterpret_cast<const char*>(normal), sizeof(normal));
        f.write(reinterpret_cast<const char*>(verts), sizeof(verts));
        f.write(reinterpret_cast<const char*>(&attr), sizeof(attr));
    }
    if (!f) throw std::runtime_error("write_stl: write failed: " + path);
}

} // namespace cfd::mesh

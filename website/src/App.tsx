import './App.css'
import screenshot2d from './assets/screenshots/screenshot-2d.webp'
import screenshotVortex from './assets/screenshots/screenshot-vortex.webp'
import screenshotStreamlines from './assets/screenshots/screenshot-streamlines.webp'

const REPO_URL = 'https://github.com/yoda-3x3/venturi-cfd'

const screenshots = [
  {
    src: screenshot2d,
    alt: 'Lid-driven cavity 2D scenario, showing the velocity magnitude field and a live convergence plot',
    caption: '2D Flow Scenarios — live velocity field + convergence plot',
  },
  {
    src: screenshotVortex,
    alt: 'Results Viewer showing a shaded, multi-colored ray-marched vortex-core (Q-criterion) volume render around an aircraft wake',
    caption: 'Built-In Results Viewer — shaded vortex-core (Q-criterion) render',
  },
  {
    src: screenshotStreamlines,
    alt: 'Results Viewer showing rainbow-colored streamlines flowing around a 3D geometry',
    caption: 'Streamlines colored by local speed',
  },
]

const features = [
  {
    title: 'No CFD Experience Needed',
    body: 'Pick one of three built-in scenarios, hit run, and watch the flow field converge live. Sensible defaults for grid resolution and Reynolds number mean you get a real result on your first try — no meshing or solver tuning required.',
  },
  {
    title: '2D Flow Scenarios',
    body: 'Lid-driven cavity, channel flow, and flow past a wall-mounted obstacle. Vorticity–streamfunction solver with a live preview of velocity, vorticity, and streamlines as it converges.',
  },
  {
    title: 'Custom 3D Geometry',
    body: 'Ready to go further? Upload an STL, OBJ, PLY, or OFF file. The mesh is centered, normalized, and voxelized onto a Cartesian grid, then solved as an immersed boundary in a virtual wind tunnel.',
  },
  {
    title: 'Cut-Cell Surface Refinement',
    body: 'Boundary cells are clipped against the actual uploaded mesh, replacing blocky voxel faces with real geometry triangles — roughly 14x lower surface deviation.',
  },
  {
    title: 'Built-In Results Viewer',
    body: 'A ray-marched vortex-core (Q-criterion) volume render, streamlines, and a movable slice plane — no need to open ParaView just to look at a run.',
  },
  {
    title: 'OpenFOAM Export',
    body: 'A full polyMesh, per-timestep U/p field files, and a .foam placeholder ParaView opens directly — for when you want the full OpenFOAM toolchain.',
  },
  {
    title: 'Smart Caching',
    body: 'Results are cached on disk keyed on geometry and settings. Identical reruns are instant, and changing only Re, steps, or thread count still reuses the voxelization.',
  },
]

function App() {
  return (
    <>
      <header className="nav">
        <div className="nav-inner">
          <span className="wordmark">
            <img src={`${import.meta.env.BASE_URL}favicon.svg`} alt="" className="wordmark-icon" />
            Venturi CFD
          </span>
          <nav>
            <a href="#features">Features</a>
            <a href="#screenshots">Screenshots</a>
            <a href="#download">Download</a>
            <a href={REPO_URL} target="_blank" rel="noreferrer">
              GitHub
            </a>
          </nav>
        </div>
      </header>

      <main>
        <section className="hero">
          <div className="hero-flow" aria-hidden="true">
            <div className="stream stream-1" />
            <div className="stream stream-2" />
            <div className="stream stream-3" />
            <div className="stream stream-4" />
          </div>
          <div className="hero-content">
            <span className="badge">Beginner friendly</span>
            <h1>CFD simulation that doesn't start with a wall of setup.</h1>
            <p className="lede">
              Venturi CFD is a desktop app for people new to computational
              fluid dynamics. Pick a built-in scenario, watch it solve with a
              live preview, and explore your own 3D geometry once you're
              ready — no Python environment, command line, or OpenFOAM
              tutorial required to get started.
            </p>
            <div className="cta-row">
              <a className="btn btn-primary" href="#download">
                Download for Windows
              </a>
              <a
                className="btn btn-secondary"
                href={REPO_URL}
                target="_blank"
                rel="noreferrer"
              >
                View source on GitHub
              </a>
            </div>
          </div>
        </section>

        <section id="features" className="features">
          <h2>What it does</h2>
          <div className="feature-grid">
            {features.map((f) => (
              <div className="feature-card" key={f.title}>
                <h3>{f.title}</h3>
                <p>{f.body}</p>
              </div>
            ))}
          </div>
        </section>

        <section id="screenshots" className="screenshots">
          <h2>See it in action</h2>
          <div className="screenshot-grid">
            {screenshots.map((s) => (
              <figure className="screenshot-card" key={s.src}>
                <img src={s.src} alt={s.alt} loading="lazy" />
                <figcaption>{s.caption}</figcaption>
              </figure>
            ))}
          </div>
        </section>

        <section id="download" className="download">
          <h2>Get Venturi CFD</h2>
          <p>
            Grab the Windows installer from the latest GitHub release, or
            build it yourself from source.
          </p>
          <div className="cta-row">
            <a
              className="btn btn-primary"
              href={`${REPO_URL}/releases`}
              target="_blank"
              rel="noreferrer"
            >
              Releases on GitHub
            </a>
            <a
              className="btn btn-secondary"
              href={`${REPO_URL}/blob/main/cfd_studio_cpp/BUILD.md`}
              target="_blank"
              rel="noreferrer"
            >
              Build from source
            </a>
          </div>
          <p className="fine-print">
            This program is still under development and does not work
            perfectly. Use it at your own risk.
          </p>
        </section>
      </main>

      <footer className="footer">
        <a href={REPO_URL} target="_blank" rel="noreferrer">
          yoda-3x3/venturi-cfd
        </a>
        <span>A personal project</span>
      </footer>
    </>
  )
}

export default App

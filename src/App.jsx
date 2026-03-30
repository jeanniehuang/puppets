import './App.css'

const shows = [
  {
    title: 'Shadow Strings',
    copy: 'A candlelit opener built around silhouettes, live foley, and hand-painted sets.',
  },
  {
    title: 'Tin Parade',
    copy: 'Clockwork birds, brass bells, and a quick comic intermission for restless crowds.',
  },
  {
    title: 'Moon Thread',
    copy: 'The final act turns the whole stage into a stitched sky with drifting marionettes.',
  },
]

const notes = [
  'Small touring company for galleries, schools, and after-hours museum events.',
  'Portable set with a 30-minute load-in and lighting plan sized for simple rooms.',
  'Original sound design, custom-built puppets, and workshop sessions for kids or adults.',
]

function App() {
  return (
    <main className="page-shell">
      <section className="hero">
        <p className="eyebrow">PUPPETS / Touring Stage Experiments</p>
        <div className="hero-grid">
          <div className="hero-copy">
            <h1>Strange little performances built from wood, thread, paper, and light.</h1>
            <p className="lede">
              Puppets is a compact performance website for a fictional troupe that mixes
              marionettes, shadow play, and miniature scenery into intimate live shows.
            </p>
            <div className="actions">
              <a href="#program" className="button button-primary">
                View program
              </a>
              <a href="#booking" className="button button-secondary">
                Book a date
              </a>
            </div>
          </div>

          <div className="stage-card" aria-hidden="true">
            <div className="stage-proscenium">
              <span className="string string-left"></span>
              <span className="string string-center"></span>
              <span className="string string-right"></span>
              <div className="curtain curtain-left"></div>
              <div className="curtain curtain-right"></div>
              <div className="puppet puppet-left">
                <span className="head"></span>
                <span className="body"></span>
              </div>
              <div className="puppet puppet-right">
                <span className="head"></span>
                <span className="body"></span>
              </div>
              <div className="stage-floor"></div>
            </div>
          </div>
        </div>
      </section>

      <section className="info-strip">
        {notes.map((note) => (
          <p key={note}>{note}</p>
        ))}
      </section>

      <section id="program" className="section-block">
        <div className="section-heading">
          <p className="eyebrow">Current Program</p>
          <h2>Three acts, one portable stage, no filler.</h2>
        </div>

        <div className="show-grid">
          {shows.map((show, index) => (
            <article key={show.title} className="show-card">
              <p className="show-index">0{index + 1}</p>
              <h3>{show.title}</h3>
              <p>{show.copy}</p>
            </article>
          ))}
        </div>
      </section>

      <section className="section-block split-layout">
        <div>
          <p className="eyebrow">What It Feels Like</p>
          <h2>Designed for rooms where the audience can hear the strings move.</h2>
        </div>
        <div className="detail-copy">
          <p>
            The visual language leans theatrical rather than corporate: warm reds, paper
            textures, and a stage frame that carries the page. It is set up as a simple
            React/Vite site so it is easy to extend into a real studio or event website.
          </p>
          <p>
            Use this as a base for ticketing, schedules, galleries, or a production
            archive. The structure is intentionally lightweight and easy to restyle.
          </p>
        </div>
      </section>

      <section id="booking" className="booking-card">
        <div>
          <p className="eyebrow">Booking</p>
          <h2>Programming a festival, storefront, or school residency?</h2>
        </div>
        <a className="button button-primary" href="mailto:bookings@puppets.example">
          bookings@puppets.example
        </a>
      </section>
    </main>
  )
}

export default App

// Logic
import{
      FaPause,
      FaPlay,
}

// dropdown & play/pause state for header button
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const [isPlaying, setIsPlaying] = useState(false);

// UI

// stopPropagation => also for zoom-in, zoom-out clicks
 {/* Dropdown with play/pause */}
        <div className="dropdown-wrap">
          <div
           role="button"
            tabIndex={0}
            onClick={() => setIsDropdownOpen((s) => !s)}
            aria-expanded={isDropdownOpen}
          > className="dropdown-btn"
            
            <button
              onClick={(e) => {
                e.stopPropagation();
                setIsPlaying((p) => !p);
              }}
              title={isPlaying ? "Pause" : "Play"}
              className={isPlaying ? "play-btn playing" : "play-btn"}
              aria-pressed={isPlaying}
            >
              {isPlaying ? <FaPause /> : <FaPlay />}
            </button>

            <span>Robot: UI EMERGENCY</span>
            <span className="caret">▾</span>
          </div>

          {isDropdownOpen && (
            <div role="dialog" className="dropdown-menu">
              <div className="dropdown-title">Robot: UI EMERGENCY</div>
              <div className="dropdown-body">
                There is no mission running. Initiate a mission to get started.
              </div>
            </div>
          )}
        </div>

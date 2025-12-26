// Lock button fully, fullscreen button isn't working

import{
      FaUnlock,
}

const [isLocked, setIsLocked] = useState(false);

const handleToggleLock = useCallback(() => {
    setIsLocked((prev) => {
      const next = !prev;
      // ensure only one toast shown by using a fixed id and dismissing previous
      toast.dismiss(LOCK_TOAST_ID);
      if (next) {
        toast.error("Console locked", { id: LOCK_TOAST_ID });
      } else {
        toast.success("Console unlocked", { id: LOCK_TOAST_ID });
      }
      return next;
    });
  }, []);

    // allow unlocking via Escape to avoid getting locked out
  useEffect(() => {
    const onKey = (e) => {
      if (!isLocked) return;
      if (e.key === "Escape") {
        handleToggleLock();
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [isLocked, handleToggleLock]);

//   Fullscreen
  const mapRef = useRef(null);

  const layoutRef = useRef(null);

// single toast id to avoid duplicates when user repeatedly clicks
  const showLockedAttemptToast = useCallback(() => {
    toast.dismiss("locked-attempt");
    toast.error("The screen is locked", { id: "locked-attempt" });
  }, []);

  return (
    
    {/* Map / Workspace placeholder - this is the element we fullscreen */}
          <div
            ref={mapRef}
            className="map-ref"
            /* ref kept on outer container so fullscreen targets the whole map area */
          >
          
    <div
        ref={layoutRef}
      className={`main-container ${rightPage ? "has-right-pane" : ""} ${
        isLocked ? "is-locked" : ""
      }`}
    >
      {/* Lock Overlay - shows when locked */}
      {isLocked && (
        <div 
          className="lock-overlay" 
          onClick={(e) => {
            e.stopPropagation();
            e.preventDefault();
          }}
        >
          <div className="unlock-hint">
            Console Locked - Click unlock button to access
          </div>
        </div>
      )}

      {/* Unlock Button - ONLY visible when locked (floating on top) */}
      {isLocked && (
        <button
          className="lock-toggle-fixed"
          onClick={handleToggleLock}
          aria-label="Unlock console"
          title="Unlock Console"
        >
          {/* Unlocked icon (padlock open) */}
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
            <rect x="3" y="11" width="18" height="11" rx="2" ry="2"></rect>
            <path d="M7 11V7a5 5 0 0 1 9.9-1"></path>
          </svg>
        </button>
      )}

        {/* Lock button in header - ONLY visible when NOT locked */}
                {!isLocked && (
                  <button
                    type="button"
                    className="lock-toggle-btn"
                    aria-pressed={false}
                    aria-label="Lock console"
                    onMouseDown={(e) => e.preventDefault()}
                    onClick={handleToggleLock}
                    title="Lock console"
                  >
                    <FaUnlock />
                  </button>
                )}
    </div>
  );

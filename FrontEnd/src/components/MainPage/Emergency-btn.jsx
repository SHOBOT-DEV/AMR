// emergency button: logic

 // stable id to avoid duplicate toasts for emergency toggle
  const EMERGENCY_TOAST_ID = "emergency-toast";

  const [emergencyClicked, setEmergencyClicked] = useState(false);
  

  // centralized handler to toggle emergency and show single toast
  const handleEmergencyToggle = React.useCallback(() => {
    setEmergencyClicked((prev) => {
      const next = !prev;
      // ensure any previous emergency toast is removed first
      toast.dismiss(EMERGENCY_TOAST_ID);
      if (next) {
        toast.error("Emergency stop engaged", { id: EMERGENCY_TOAST_ID });
      } else {
        toast.success("Emergency stop released", { id: EMERGENCY_TOAST_ID });
      }
      return next;
    });
  }, []);


// UI
    return (
        <div className="emergency-button">
               {/* Header-centered Emergency Stop */}
            <div className="header-center-emergency">
            <button
                onClick={handleEmergencyToggle}
                aria-pressed={emergencyClicked}
                className={
                emergencyClicked
                    ? "header-emergency-btn clicked"
                    : "header-emergency-btn"
                }
            >
                <span className="header-emergency-dot" aria-hidden="true">
                <span className="header-emergency-dot-inner" />
                </span>
                <span className="header-emergency-label">Emergency Stop</span>
            </button>
            </div>
        </div>
    );

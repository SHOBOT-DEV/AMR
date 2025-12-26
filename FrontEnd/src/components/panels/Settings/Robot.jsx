import { useState } from "react";

const [robotSettingsState, setRobotSettingsState] = useState({
    autopilot: true,
    safeMode: true,
    remoteDiagnostics: false,
    pathOptimization: true,
  });


import React, { useState } from "react";

interface RobotSettings {
  autopilot: boolean;
  safeMode: boolean;
  remoteDiagnostics: boolean;
  pathOptimization: boolean;
  [key: string]: boolean;
}

interface RobotProps {
  robotSettingsState?: RobotSettings;
  setRobotSettingsState?: (settings: RobotSettings) => void;
}

const Robot: React.FC<RobotProps> = ({
  robotSettingsState: externalSettings,
  setRobotSettingsState: externalSetSettings,
}) => {
  const [settings, setSettings] = useState<RobotSettings>({
    autopilot: true,
    safeMode: true,
    remoteDiagnostics: false,
    pathOptimization: true,
  });

  // Use external settings if provided, otherwise use local state
  const robotSettings = externalSettings || settings;
  const setRobotSettings = externalSetSettings || setSettings;

  // Get user-friendly label from camelCase key
  const getLabel = (key: string): string => {
    return key.replace(/([A-Z])/g, " $1").trim();
  };

  // Get description based on setting key
  const getDescription = (key: string): string => {
    const descriptions: Record<string, string> = {
      autopilot: "Autonomous navigation mode",
      safeMode: "Collision avoidance enabled",
      remoteDiagnostics: "Allow remote system monitoring",
      pathOptimization: "Intelligent route planning",
    };
    return descriptions[key] || `Control ${getLabel(key).toLowerCase()}`;
  };

  const toggleSetting = (key: string) => {
    setRobotSettings({
      ...robotSettings,
      [key]: !robotSettings[key],
    });
  };

  return (
    <div className="flex flex-col gap-3">
      {Object.entries(robotSettings).map(([key, value]) => (
        <div
          key={key}
          className="flex items-center justify-between p-4 border border-slate-200 rounded-lg bg-white hover:bg-slate-50 transition-colors"
        >
          <div className="flex-1">
            <h4 className="text-sm font-semibold text-slate-900 capitalize">
              {getLabel(key)}
            </h4>
            <p className="text-xs text-slate-600 mt-1">
              {getDescription(key)}
            </p>
            <span
              className={`inline-block text-xs font-medium mt-2 px-2 py-1 rounded-full ${
                value
                  ? "bg-green-100 text-green-700"
                  : "bg-slate-100 text-slate-600"
              }`}
            >
              {value ? "Enabled" : "Disabled"}
            </span>
          </div>

          {/* Toggle Switch */}
          <button
            onClick={() => toggleSetting(key)}
            className={`ml-4 relative w-12 h-6 rounded-full transition-all flex-shrink-0 ${
              value ? "bg-sky-500" : "bg-slate-300"
            }`}
            role="switch"
            aria-checked={value}
            aria-label={`Toggle ${getLabel(key)}`}
          >
            <div
              className={`w-4 h-4 bg-white rounded-full absolute top-1 transition-all ${
                value ? "left-7" : "left-1"
              }`}
              aria-hidden="true"
            />
          </button>
        </div>
      ))}
    </div>
  );
};

export default Robot;
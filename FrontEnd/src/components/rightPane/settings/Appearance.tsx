import React, { useState } from "react";

type ThemeOption = "light" | "dark" | "system";

interface Theme {
  id: ThemeOption;
  label: string;
  description: string;
}

interface AppearanceProps {
  selectedTheme?: ThemeOption;
  setSelectedTheme?: (theme: ThemeOption) => void;
}

const Appearance: React.FC<AppearanceProps> = ({
  selectedTheme: externalTheme,
  setSelectedTheme: externalSetTheme,
}) => {
  const [theme, setTheme] = useState<ThemeOption>("light");

  // Use external theme if provided, otherwise use local state
  const selectedTheme = externalTheme || theme;
  const setSelectedTheme = externalSetTheme || setTheme;

  const themes: Theme[] = [
    {
      id: "light",
      label: "Light",
      description: "Bright and clean interface",
    },
    {
      id: "dark",
      label: "Dark",
      description: "Easy on the eyes in low light",
    },
    {
      id: "system",
      label: "Match System",
      description: "Follow device preferences",
    },
  ];

  const handleThemeSelect = (themeId: ThemeOption) => {
    setSelectedTheme(themeId);
  };

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-3">
      {themes.map((themeOption) => {
        const isSelected = selectedTheme === themeOption.id;
        return (
          <button
            key={themeOption.id}
            onClick={() => handleThemeSelect(themeOption.id)}
            className={`p-4 rounded-lg border-2 transition-all text-left flex flex-col gap-2 hover:scale-[1.02] ${
              isSelected
                ? "border-blue-500 bg-blue-50 shadow-md"
                : "border-slate-200 bg-white hover:border-blue-300"
            }`}
          >
            <div className="flex items-center justify-between">
              <strong className="text-lg font-semibold text-slate-900">
                {themeOption.label}
              </strong>
              {isSelected && (
                <div className="flex items-center justify-center w-6 h-6 rounded-full bg-blue-500">
                  <svg
                    className="w-4 h-4 text-white"
                    fill="none"
                    strokeLinecap="round"
                    strokeLinejoin="round"
                    strokeWidth="2"
                    viewBox="0 0 24 24"
                    stroke="currentColor"
                  >
                    <path d="M5 13l4 4L19 7"></path>
                  </svg>
                </div>
              )}
            </div>
            <span className="text-sm text-slate-600">
              {themeOption.description}
            </span>
            <span
              className={`text-xs font-medium mt-1 ${
                isSelected ? "text-blue-600" : "text-slate-400"
              }`}
            >
              {isSelected ? "Selected" : "Click to use"}
            </span>
          </button>
        );
      })}
    </div>
  );
};

export default Appearance;
import React, { useMemo, useRef } from "react";

import map1 from "../../assets/map1.png";
import test1 from "../../assets/test_1.png";
import shobotLogo from "../../assets/shobot_arena.png";
import shobotLogo1 from "../../assets/shobot_arena2.png";
import simulationMap from "../../assets/simulation_map.png";
import trasccon4th from "../../assets/trasccon4th.png";

interface MapAreaProps {
  minimized?: boolean;
  zoomLevel?: number;
  selectedMap?: { id?: string; image?: string; name?: string } | null;
  toggleMapZoom?: () => void;
  onJoystickMove?: (data: any) => void;
}

const MapArea: React.FC<MapAreaProps> = ({
  minimized = false,
  zoomLevel = 1,
  selectedMap,
  toggleMapZoom,
  onJoystickMove,
}) => {
  const mapRef = useRef<HTMLDivElement | null>(null);

  // Normalize ids coming from API/UI so fallbacks match expected assets
  const mapIdAliases: Record<string, string> = {
    shobot_area: "shobot_arena",
    arena: "shobot_arena",
    trasccon: "trasccon4th",
  };
  const normalizeId = (id?: string) => mapIdAliases[String(id || "").toLowerCase()] || (id || "");

  // Known map images to use as fallbacks when the provided src is missing/broken
  const defaultMapsWithImages = useMemo(
    () => [
      { id: "shobot_arena", name: "Shobot Arena", image: shobotLogo },
      { id: "shobot_arena2", name: "Shobot Arena 2", image: shobotLogo1 },
      { id: "simulation", name: "Simulation Map", image: simulationMap },
      { id: "trasccon4th", name: "Trasccon 4th Floor", image: trasccon4th },
      { id: "map1", name: "Map 1", image: map1 },
      { id: "test_1", name: "Test 1", image: test1 },
    ],
    []
  );

  const defaultImageById = useMemo(
    () => Object.fromEntries(defaultMapsWithImages.map((dm) => [String(dm.id || "").toLowerCase(), dm.image])),
    [defaultMapsWithImages]
  );
  const defaultImageByName = useMemo(
    () => Object.fromEntries(defaultMapsWithImages.map((dm) => [String(dm.name || "").toLowerCase(), dm.image])),
    [defaultMapsWithImages]
  );

  const getFallbackImage = (id?: string, name?: string) => {
    const normId = normalizeId(id);
    return defaultImageById[String(normId || "").toLowerCase()] || defaultImageByName[String(name || "").toLowerCase()] || "";
  };

  const resolvedImage = useMemo(() => {
    if (!selectedMap) return "";
    const explicit = (selectedMap.image || "").trim();
    return explicit || getFallbackImage(selectedMap.id as string, selectedMap.name);
  }, [selectedMap, defaultImageById, defaultImageByName]);

  return (
    <main
      ref={mapRef}
      className={`relative h-full w-full overflow-hidden bg-white transition-all duration-300 ${
        minimized ? "minimized opacity-75" : ""
      }`}
      style={{
        transform: minimized ? "scale(0.95)" : "scale(1)",
      }}
    >
      {/* Map content */}
      <div
        className="h-full w-full flex items-center justify-center cursor-pointer select-none"
        onClick={toggleMapZoom}
        role="button"
        tabIndex={0}
        onKeyDown={(e) => {
          if (e.key === "Enter" || e.key === " ") toggleMapZoom?.();
        }}
        style={{
          transform: `scale(${1 + (zoomLevel - 1)})`,
          transition: "transform 280ms ease",
        }}
      >
        {resolvedImage ? (
          <img
            src={resolvedImage}
            alt={selectedMap.name || "Map preview"}
            style={{
              width: "100%",
              height: "100%",
              objectFit: "contain",
              display: "block",
            }}
            onError={(e) => {
              const fallback = getFallbackImage(selectedMap?.id as string, selectedMap?.name);
              if (fallback && e.currentTarget.src !== fallback) {
                e.currentTarget.src = fallback;
                e.currentTarget.onerror = null;
              } else {
                e.currentTarget.style.display = "none";
              }
            }}
          />
        ) : (
          <div
            className="w-full h-full bg-gradient-to-br from-slate-100 to-slate-200 flex items-center justify-center"
            aria-hidden="true"
          >
            <span className="text-slate-400 text-lg font-semibold">
              {selectedMap?.name || "No map selected"}
            </span>
          </div>
        )}
      </div>
    </main>
  );
};

export default MapArea;

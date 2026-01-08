import React, { useEffect, useMemo } from "react";
import { toast } from "react-hot-toast";
import { useCreate } from "../../../context/CreateContext.tsx";

/* ================= TYPES ================= */

interface MapItem {
  id: string;
  name: string;
  status?: string;
}

interface Waypoint {
  id: string;
  mapId: string;
  name: string;
  category: string;
  geom: string;
  notes: string;
  active: boolean;
  createdAt: string;
}

interface Zone {
  id: string;
  mapId: string;
  name: string;
  category: string;
  geometry: string;
  active: boolean;
  createdAt: string;
}

interface Mission {
  id: string;
  mapId: string;
  name: string;
  owner: string;
  status: string;
  notes: string;
  createdAt: string;
  iteration: number;
}

type RenderProp = (
  selectedMap: MapItem | null,
  waypoints: Waypoint[],
  zones: Zone[],
  missions: Mission[]
) => React.ReactNode;

interface MapContextProps {
  children: RenderProp;
}

/* ================= CONTEXT ================= */

const MapContext: React.FC<MapContextProps> = ({ children }) => {
  const {
    selectedMap,
    waypoints,
    setWaypoints,
    zones,
    setZones,
    missions,
    setMissions,
  } = useCreate();

  /* ========== CREATE DEFAULT DATA (ONCE PER MAP) ========== */

  useEffect(() => {
    if (!selectedMap) return;

    const mapId = selectedMap.id;

    if (!waypoints.some((w) => w.mapId === mapId)) {
      setWaypoints((prev) => [
        ...prev,
        {
          id: `wp-${mapId}-1`,
          mapId,
          name: "Delivery",
          category: "Normal",
          geom: "Point(10 10)",
          notes: `Default waypoint for ${selectedMap.name}`,
          active: true,
          createdAt: new Date().toLocaleString(),
        },
      ]);
      toast.success(`Waypoints created for ${selectedMap.name}`);
    }

    if (!zones.some((z) => z.mapId === mapId)) {
      setZones((prev) => [
        ...prev,
        {
          id: `zone-${mapId}-1`,
          mapId,
          name: "Safe Zone",
          category: "Safe",
          geometry: "Polygon((0,0),(100,0),(100,50),(0,50))",
          active: true,
          createdAt: new Date().toLocaleString(),
        },
      ]);
      toast.success(`Zones created for ${selectedMap.name}`);
    }

    if (!missions.some((m) => m.mapId === mapId)) {
      setMissions((prev) => [
        ...prev,
        {
          id: `mission-${mapId}-1`,
          mapId,
          name: "Delivery Mission",
          owner: "System",
          status: "Draft",
          notes: `Default mission for ${selectedMap.name}`,
          createdAt: new Date().toLocaleString(),
          iteration: 1,
        },
      ]);
      toast.success(`Missions created for ${selectedMap.name}`);
    }
  }, [selectedMap]); // ✅ CRITICAL FIX

  /* ========== FILTERED DATA ========== */

  const filteredWaypoints = useMemo(
    () => waypoints.filter((w) => w.mapId === selectedMap?.id),
    [waypoints, selectedMap]
  );

  const filteredZones = useMemo(
    () => zones.filter((z) => z.mapId === selectedMap?.id),
    [zones, selectedMap]
  );

  const filteredMissions = useMemo(
    () => missions.filter((m) => m.mapId === selectedMap?.id),
    [missions, selectedMap]
  );

  return (
    <>
      {children(
        selectedMap,
        filteredWaypoints,
        filteredZones,
        filteredMissions
      )}
    </>
  );
};

export default MapContext;

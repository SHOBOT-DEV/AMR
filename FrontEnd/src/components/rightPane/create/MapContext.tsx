import React, { useState } from "react";
import { toast } from "react-hot-toast";

// Type definitions
interface Map {
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

// Props for component
interface MapManagerProps {
  mapsList: Map[];
  waypoints: Waypoint[];
  setWaypoints: React.Dispatch<React.SetStateAction<Waypoint[]>>;
  zones: Zone[];
  setZones: React.Dispatch<React.SetStateAction<Zone[]>>;
  missions: Mission[];
  setMissions: React.Dispatch<React.SetStateAction<Mission[]>>;
  requestV1: (url: string, options: RequestInit) => Promise<any>;
}

const MapManager: React.FC<MapManagerProps> = ({
  mapsList,
  waypoints,
  setWaypoints,
  zones,
  setZones,
  missions,
  setMissions,
  requestV1,
}) => {
  const [selectedMap, setSelectedMap] = useState<Map | null>(null);
  const [rightPage, setRightPage] = useState<string | null>(null);

  // Filtered getters
  const getFilteredWaypoints = (): Waypoint[] => {
    if (!selectedMap) return [];
    const filtered = waypoints.filter((wp) => wp.mapId === selectedMap.id);
    console.log(`📍 getFilteredWaypoints for map ${selectedMap.id}:`, filtered);
    return filtered;
  };

  const getFilteredMissions = (): Mission[] => {
    if (!selectedMap) return [];
    const filtered = missions.filter((m) => m.mapId === selectedMap.id);
    console.log(`📍 getFilteredMissions for map ${selectedMap.id}:`, filtered);
    return filtered;
  };

  const handleRightPageChange = (id: string) => {
    if (id === "chat") {
      setRightPage("chat");
    } else if (id === "stats") {
      setRightPage("stats");
    } else if (
      createIds.has(id) ||
      monitorIds.has(id) ||
      settingsIds.has(id)
    ) {
      setRightPage(id);
      let activeMap: Map | undefined;
      if (id === "maps") {
        setSelectedMap(mapsList[0]);
      } else if (id === "zones" || id === "waypoints" || id === "missions") {
        activeMap = mapsList.find(
          (m) => (m.status || "").toLowerCase() === "active"
        );
        if (activeMap) setSelectedMap(activeMap);
      }
      // Users page does not need map selection
    } else {
      setRightPage(null);
    }
  };

  const activateMap = async (map: Map) => {
    try {
      setSelectedMap({ ...map, status: "Active" });
      const mapId = map.id;

      const hasWaypoints = waypoints.some((wp) => wp.mapId === mapId);
      const hasZones = zones.some((z) => z.mapId === mapId);
      const hasMissions = missions.some((m) => m.mapId === mapId);

      console.log(
        `📍 Existing data - Waypoints: ${hasWaypoints}, Zones: ${hasZones}, Missions: ${hasMissions}`
      );

      if (!hasWaypoints) {
        const defaultWaypoints: Waypoint[] = [
          {
            id: `wp-${mapId}-delivery`,
            mapId,
            name: "Delivery",
            category: "Normal",
            geom: "Point(10 10)",
            notes: `Delivery waypoint for ${map.name}`,
            active: true,
            createdAt: new Date().toLocaleString(),
          },
          {
            id: `wp-${mapId}-shelf1`,
            mapId,
            name: "Shelf_1",
            category: "Normal",
            geom: "Point(50 30)",
            notes: `Shelf 1 waypoint for ${map.name}`,
            active: true,
            createdAt: new Date().toLocaleString(),
          },
          {
            id: `wp-${mapId}-shelf2`,
            mapId,
            name: "Shelf_2",
            category: "Normal",
            geom: "Point(80 50)",
            notes: `Shelf 2 waypoint for ${map.name}`,
            active: true,
            createdAt: new Date().toLocaleString(),
          },
          {
            id: `wp-${mapId}-shelf3`,
            mapId,
            name: "Shelf_3",
            category: "Normal",
            geom: "Point(20 70)",
            notes: `Shelf 3 waypoint for ${map.name}`,
            active: true,
            createdAt: new Date().toLocaleString(),
          },
          {
            id: `wp-${mapId}-shelf4`,
            mapId,
            name: "Shelf_4",
            category: "Normal",
            geom: "Point(60 80)",
            notes: `Shelf 4 waypoint for ${map.name}`,
            active: true,
            createdAt: new Date().toLocaleString(),
          },
        ];
        console.log(`✅ Creating ${defaultWaypoints.length} waypoints for map ${mapId}`);
        setWaypoints((prev) => [...prev, ...defaultWaypoints]);
        toast.success(`Created 5 waypoints for ${map.name}`);
      }

      if (!hasZones) {
        const defaultZones: Zone[] = [
          {
            id: `zone-${mapId}-1`,
            mapId,
            name: `${map.name} - Safe Zone`,
            category: "Safe",
            geometry: "Polygon((0,0),(100,0),(100,50),(0,50))",
            active: true,
            createdAt: new Date().toLocaleString(),
          },
          {
            id: `zone-${mapId}-2`,
            mapId,
            name: `${map.name} - Caution Area`,
            category: "Caution",
            geometry: "Polygon((0,50),(50,50),(50,100),(0,100))",
            active: true,
            createdAt: new Date().toLocaleString(),
          },
          {
            id: `zone-${mapId}-3`,
            mapId,
            name: `${map.name} - Restricted`,
            category: "No-Go",
            geometry: "Polygon((50,50),(100,50),(100,100),(50,100))",
            active: true,
            createdAt: new Date().toLocaleString(),
          },
        ];
        console.log(`✅ Creating ${defaultZones.length} zones for map ${mapId}`);
        setZones((prev) => [...prev, ...defaultZones]);
        toast.success(`Created 3 zones for ${map.name}`);
      }

      if (!hasMissions) {
        const defaultMissions: Mission[] = [
          {
            id: `mission-${mapId}-delivery`,
            mapId,
            name: "Delivery",
            owner: "CNDE",
            status: "Draft",
            notes: `Delivery mission for ${map.name}`,
            createdAt: new Date().toLocaleString(),
            iteration: 1,
          },
          {
            id: `mission-${mapId}-shelf1`,
            mapId,
            name: "Shelf_1",
            owner: "CNDE",
            status: "Draft",
            notes: `Shelf 1 mission for ${map.name}`,
            createdAt: new Date().toLocaleString(),
            iteration: 1,
          },
          {
            id: `mission-${mapId}-shelf2`,
            mapId,
            name: "Shelf_2",
            owner: "CNDE",
            status: "Draft",
            notes: `Shelf 2 mission for ${map.name}`,
            createdAt: new Date().toLocaleString(),
            iteration: 1,
          },
          {
            id: `mission-${mapId}-shelf3`,
            mapId,
            name: "Shelf_3",
            owner: "CNDE",
            status: "Draft",
            notes: `Shelf 3 mission for ${map.name}`,
            createdAt: new Date().toLocaleString(),
            iteration: 1,
          },
          {
            id: `mission-${mapId}-shelf4`,
            mapId,
            name: "Shelf_4",
            owner: "CNDE",
            status: "Draft",
            notes: `Shelf 4 mission for ${map.name}`,
            createdAt: new Date().toLocaleString(),
            iteration: 1,
          },
        ];
        console.log(`✅ Creating ${defaultMissions.length} missions for map ${mapId}`);
        setMissions((prev) => [...prev, ...defaultMissions]);
        toast.success(`Created 5 missions for ${map.name}`);
      }

      toast.success(`Activated map: ${map.name}`);

      // Persist change to server
      try {
        await requestV1(`/maps/${map.id}`, {
          method: "PATCH",
          body: JSON.stringify({ status: "Active" }),
        });
      } catch (err) {
        console.warn("Failed to persist map activation:", err);
      }
    } catch (err) {
      console.error("Activation error", err);
      toast.error("Failed to activate map");
    }
  };

  return (
    <div>
      {/* Component UI goes here */}
    </div>
  );
};

export default MapManager;

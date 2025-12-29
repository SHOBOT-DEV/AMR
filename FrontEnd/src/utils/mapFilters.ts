/**
 * Map-dependent filtering utilities for Waypoints, Zones, and Missions
 * Centralizes logic that filters these entities based on the currently selected map
 */

export type MapRecord = {
  id: string | number | null;
  name?: string;
};

export type WaypointRecord = {
  id: string | number;
  mapId: string | number;
  name: string;
  category: string;
  geom: string;
  notes?: string;
  active: boolean;
  createdAt?: string;
};

export type ZoneRecord = {
  id: string | number;
  mapId: string | number;
  name: string;
  category: string;
  geometry: string;
  active: boolean;
  createdAt?: string;
};

export type MissionRecord = {
  id?: string | number | null;
  mapId?: string | number | null;
  name: string;
  owner: string;
  status: string;
  createdAt?: string;
  notes?: string;
};

/**
 * Filter waypoints based on selected map
 * @param waypoints - All waypoints array
 * @param selectedMap - Currently selected map
 * @returns Array of waypoints that belong to the selected map
 */
export const getFilteredWaypoints = (
  waypoints: WaypointRecord[],
  selectedMap: MapRecord | null | undefined
): WaypointRecord[] => {
  if (!selectedMap?.id) return [];
  const filtered = waypoints.filter((wp) => wp.mapId === selectedMap.id);
  console.log(`📍 getFilteredWaypoints for map ${selectedMap.id}:`, filtered);
  return filtered;
};

/**
 * Filter zones based on selected map
 * @param zones - All zones array
 * @param selectedMap - Currently selected map
 * @returns Array of zones that belong to the selected map
 */
export const getFilteredZones = (
  zones: ZoneRecord[],
  selectedMap: MapRecord | null | undefined
): ZoneRecord[] => {
  if (!selectedMap?.id) return [];
  const filtered = zones.filter((z) => z.mapId === selectedMap.id);
  console.log(`📍 getFilteredZones for map ${selectedMap.id}:`, filtered);
  return filtered;
};

/**
 * Filter missions based on selected map
 * @param missions - All missions array
 * @param selectedMap - Currently selected map
 * @returns Array of missions that belong to the selected map
 */
export const getFilteredMissions = (
  missions: MissionRecord[],
  selectedMap: MapRecord | null | undefined
): MissionRecord[] => {
  if (!selectedMap?.id) return [];
  const filtered = missions.filter((m) => m.mapId === selectedMap.id);
  console.log(`📍 getFilteredMissions for map ${selectedMap.id}:`, filtered);
  return filtered;
};

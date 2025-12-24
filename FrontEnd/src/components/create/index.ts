import { FaFlag, FaMapMarkedAlt, FaMapMarkerAlt, FaUsers, FaLayerGroup } from "react-icons/fa";
import MapsPanel, { SAMPLE_MAPS, useSampleMapState, type MapRecord } from "./Maps.tsx";
import MissionsPanel from "./Missions";
import ZonesPanel from "./zones";
import WaypointsPanel from "./Waypoints";
import UsersPanel from "./users";

export const CREATE_MENU_ITEMS = [
  { id: "maps", label: "Maps", icon: FaMapMarkedAlt },
  { id: "zones", label: "Zones", icon: FaLayerGroup },
  { id: "waypoints", label: "Waypoints", icon: FaMapMarkerAlt },
  { id: "missions", label: "Missions", icon: FaFlag },
  { id: "users", label: "Users", icon: FaUsers },
];

export { MapsPanel, MissionsPanel, ZonesPanel, WaypointsPanel, UsersPanel, SAMPLE_MAPS, useSampleMapState };
export type { MapRecord };

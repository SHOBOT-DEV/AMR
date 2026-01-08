import React, { createContext, useContext, useState } from "react";

// Sample data moved here so it can be shared via context
const SAMPLE_MAPS = [
    { id: "cfl_gf", name: "CFL_GF", createdBy: "CNDE IITM", image: "", status: "Active", category: "Production", createdAt: "2025-11-17" },
    { id: "arena", name: "Shobot Arena", createdBy: "ANSCER ADMIN", image: "/images/maps/shobot_arena.png", status: "Draft", category: "Testing", createdAt: "2025-11-16" },
];

const SAMPLE_WAYPOINTS = [
    { id: "wp1", mapId: "cfl_gf", name: "WP_A", category: "Nav", active: true, geom: "Point(40 12)", createdAt: "2025-11-17", notes: "Primary pickup" },
    { id: "wp2", mapId: "cfl_gf", name: "WP_B", category: "Inspect", active: false, geom: "Point(98 76)", createdAt: "2025-11-17", notes: "Inspection point" },
];

const SAMPLE_ZONES = [
    { id: "z1", mapId: "arena", name: "Dock Corridor", category: "Safe", active: true, geometry: "Polygon(88,44…)", createdAt: "2025-11-16" },
    { id: "z2", mapId: "arena", name: "Battery Corner", category: "No-Go", active: true, geometry: "Polygon(27,11…)", createdAt: "2025-11-15" },
];

const SAMPLE_MISSIONS = [
    { id: "m1", mapId: "cfl_gf", name: "Inspect Zone A", owner: "CNDE", status: "Draft", createdAt: "2025-11-17", notes: "Routine inspection" },
    { id: "m2", mapId: "cfl_gf", name: "Delivery Route 3", owner: "ANSCER ADMIN", status: "Scheduled", createdAt: "2025-11-16", notes: "Delivery to docks" },
];

const SAMPLE_USERS = [
    { id: 1, username: "john_doe", email: "john.doe@example.com", company: "ANSCER Robotics", amr_type: "Type A", role: "user", approval: "Approved" },
    { id: 2, username: "jane_smith", email: "jane.smith@example.com", company: "CNDE IITM", amr_type: "Type B", role: "user", approval: "Pending" },
    { id: 3, username: "alice_johnson", email: "alice.johnson@example.com", company: "Innovation Labs", amr_type: "Type C", role: "user", approval: "Rejected" },
];

type CreateContextShape = {
    mapsList: any[];
    setMapsList: React.Dispatch<React.SetStateAction<any[]>>;
    selectedMap: any | null;
    setSelectedMap: React.Dispatch<React.SetStateAction<any | null>>;
    waypoints: any[];
    setWaypoints: React.Dispatch<React.SetStateAction<any[]>>;
    zones: any[];
    setZones: React.Dispatch<React.SetStateAction<any[]>>;
    missions: any[];
    setMissions: React.Dispatch<React.SetStateAction<any[]>>;
    users: any[];
    setUsers: React.Dispatch<React.SetStateAction<any[]>>;
};

const CreateContext = createContext<CreateContextShape | undefined>(undefined);

export const CreateProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
    const [mapsList, setMapsList] = useState<any[]>(SAMPLE_MAPS);
    const [selectedMap, setSelectedMap] = useState<any>(SAMPLE_MAPS[1] || null);
    const [waypoints, setWaypoints] = useState<any[]>(SAMPLE_WAYPOINTS);
    const [zones, setZones] = useState<any[]>(SAMPLE_ZONES);
    const [missions, setMissions] = useState<any[]>(SAMPLE_MISSIONS);
    const [users, setUsers] = useState<any[]>(SAMPLE_USERS);

    return (
        <CreateContext.Provider value={{ mapsList, setMapsList, selectedMap, setSelectedMap, waypoints, setWaypoints, zones, setZones, missions, setMissions, users, setUsers }}>
            {children}
        </CreateContext.Provider>
    );
};

export const useCreate = (): CreateContextShape => {
    const ctx = useContext(CreateContext);
    if (!ctx) throw new Error("useCreate must be used within CreateProvider");
    return ctx;
};
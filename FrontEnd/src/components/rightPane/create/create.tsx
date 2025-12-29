//   // Add filtered getters for waypoints, missions based on active map
//   const getFilteredWaypoints = () => {
//     if (!selectedMap) return [];
//     const filtered = waypoints.filter((wp) => wp.mapId === selectedMap.id);
//     console.log(`📍 getFilteredWaypoints for map ${selectedMap.id}:`, filtered);
//     return filtered;
//   };

//   const getFilteredMissions = () => {
//     if (!selectedMap) return [];
//     const filtered = missions.filter((m) => m.mapId === selectedMap.id);
//     console.log(`📍 getFilteredMissions for map ${selectedMap.id}:`, filtered);
//     return filtered;
//   };
//     if (id === "chat") {
//       setRightPage("chat");
//     } else if (id === "stats") {
//       setRightPage("stats");
//     } else if (createIds.has(id) || monitorIds.has(id) || settingsIds.has(id)) {
//       setRightPage(id);
//       // if it's the maps "page", preselect first map
//       if (id === "maps") {
//         setSelectedMap(mapsList[0]);
//       } else if (id === "zones") {
//         // Select the active map (not the "zones" dummy entry)
//         const activeMap = mapsList.find((m) => (m.status || "").toLowerCase() === "active");
//         if (activeMap) setSelectedMap(activeMap);
//       } else if (id === "waypoints") {
//         // Select the active map so waypoints page shows that map's waypoints
//         const activeMap = mapsList.find((m) => (m.status || "").toLowerCase() === "active");
//         if (activeMap) setSelectedMap(activeMap);
//       } else if (id === "missions") {
//         // Select the active map so missions page shows that map's missions
//         const activeMap = mapsList.find((m) => (m.status || "").toLowerCase() === "active");
//         if (activeMap) setSelectedMap(activeMap);
//       } else if (id === "users") {
//         // Users page doesn't need a selectedMap — it shows all users
//         // Don't change selectedMap so it stays on the current map
//         // RightPane will handle the users display directly without map filtering
//       }
//     } else {
//       // ignore main sidebar icons (do not render a page)
//       setRightPage(null);
//     }
//   };
 
//   // Create and set zones immediately (moved to Zones.tsx)

//       // Show success toasts
//       if (!hasWaypoints) {
//         toast.success(`Created 5 waypoints for ${map.name}`);
//       }
//       if (!hasMissions) {
//         toast.success(`Created 5 missions for ${map.name}`);
//       }
//       toast.success(`Activated map: ${map.name}`);

//       // Persist change to server (best-effort)
//       try {
//         await requestV1(`/maps/${map.id}`, {
//           method: "PATCH",
//           body: JSON.stringify({ status: "Active" }),
//         });
//       } catch (err) {
//         console.warn("Failed to persist map activation:", err);
//       }
//     } catch (err) {
//       console.error("Activation error", err);
//       toast.error("Failed to activate map");
//     }
//   };
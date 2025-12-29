# Map-Dependent Filtering Logic - Implementation Summary

## Overview
Extracted and centralized the map-dependent filtering logic for Waypoints, Zones, and Missions into a shared utility module.

## Changes Made

### 1. Created Centralized Utility Module
**File**: `src/utils/mapFilters.ts`

This new file contains:
- **Type Definitions**: `MapRecord`, `WaypointRecord`, `ZoneRecord`, `MissionRecord`
- **Three Filtering Functions**:
  - `getFilteredWaypoints()` - Filters waypoints by selected map
  - `getFilteredZones()` - Filters zones by selected map
  - `getFilteredMissions()` - Filters missions by selected map

Each function:
- Takes the full data array and selected map as parameters
- Returns only items matching the selected map's ID
- Includes console logging for debugging
- Handles null/undefined maps gracefully

### 2. Updated Component Files

#### `src/components/rightPane/create/zones.tsx`
- Imports `getFilteredZones` and types from `mapFilters.ts`
- Removed duplicate type definitions
- Updated `zonesForMap` useMemo to use `getFilteredZones()` utility

#### `src/components/rightPane/create/Waypoints.tsx`
- Imports `getFilteredWaypoints` and types from `mapFilters.ts`
- Removed duplicate type definitions
- Updated `filteredWaypoints` useMemo to use `getFilteredWaypoints()` utility
- Search logic remains in component for filtering

#### `src/components/rightPane/create/Missions.tsx`
- Imports `getFilteredMissions` and types from `mapFilters.ts`
- Removed duplicate type definitions
- Updated `missionsForMap` useMemo to use `getFilteredMissions()` utility

## Benefits

1. **Single Source of Truth**: Filtering logic is defined once in `mapFilters.ts`
2. **Consistency**: All three components use the same filtering approach
3. **Maintainability**: Changes to filtering logic only need to be made in one place
4. **Reusability**: Other components can easily import and use these utilities
5. **Type Safety**: Centralized type definitions prevent inconsistencies

## Usage Example

```tsx
import { 
  getFilteredWaypoints, 
  getFilteredZones, 
  getFilteredMissions,
  type WaypointRecord,
  type ZoneRecord,
  type MissionRecord,
  type MapRecord
} from "../utils/mapFilters";

// In your component
const filteredWaypoints = getFilteredWaypoints(waypoints, selectedMap);
const filteredZones = getFilteredZones(zones, selectedMap);
const filteredMissions = getFilteredMissions(missions, selectedMap);
```

## Map Dependency Logic

The filtering logic works as follows:
1. Checks if `selectedMap` exists and has an `id`
2. If map is not selected, returns empty array
3. If map is selected, filters array items where `mapId === selectedMap.id`
4. Logs filtered results for debugging

This ensures that when users switch maps in the UI, all waypoints, zones, and missions automatically update to show only items belonging to the selected map.

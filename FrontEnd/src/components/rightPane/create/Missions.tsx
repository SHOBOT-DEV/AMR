import React, { useEffect, useMemo, useState } from "react";
import { FaPlus } from "react-icons/fa";
import { getFilteredMissions, type MissionRecord, type MapRecord } from "../../../utils/mapFilters.ts";

type ToastApi = {
  success?: (message: string) => void;
  error?: (message: string) => void;
};

type Props = {
  missions: MissionRecord[];
  mapsList?: MapRecord[];
  selectedMap?: MapRecord | null;
  setMissions: React.Dispatch<React.SetStateAction<MissionRecord[]>>;
  requestV1?: (path: string, options?: RequestInit) => Promise<any>;
  toast?: ToastApi;
};

type MissionFormState = {
  name: string;
  owner: string;
  status: string;
  notes: string;
};

const defaultFormState: MissionFormState = {
  name: "",
  owner: "",
  status: "Draft",
  notes: "",
};

const statusPillClasses = (status: string) => {
  if (status === "Completed") {
    return "bg-emerald-100 text-emerald-700";
  }
  if (status === "In Progress") {
    return "bg-sky-100 text-sky-700";
  }
  if (status === "Scheduled") {
    return "bg-amber-100 text-amber-700";
  }
  return "bg-red-100 text-red-700";
};

const MissionsPanel: React.FC<Props> = ({
  missions,
  mapsList = [],
  selectedMap,
  setMissions,
  requestV1,
  toast,
}) => {
  const [missionFormOpen, setMissionFormOpen] = useState(false);
  const [missionForm, setMissionForm] = useState<MissionFormState>(defaultFormState);
  const [missionSearchField, setMissionSearchField] = useState("any");
  const [missionSearchTerm, setMissionSearchTerm] = useState("");
  const [selectedMissionId, setSelectedMissionId] = useState<string | number | null>(null);

  const missionsForMap = useMemo(() => {
    return getFilteredMissions(missions, selectedMap);
  }, [missions, selectedMap]);

  const filteredMissions = useMemo(() => {
    const term = missionSearchTerm.trim().toLowerCase();
    if (!term) return missionsForMap;
    if (missionSearchField === "any") {
      return missionsForMap.filter((mission) =>
        [mission.name, mission.owner, mission.status, mission.createdAt]
          .map((value) => (value || "").toString().toLowerCase())
          .some((value) => value.includes(term)),
      );
    }
    return missionsForMap.filter((mission) =>
      String((mission as any)[missionSearchField] || "").toLowerCase().includes(term),
    );
  }, [missionSearchField, missionSearchTerm, missionsForMap]);

  useEffect(() => {
    if (!missionsForMap.length) {
      setSelectedMissionId(null);
      return;
    }
    setSelectedMissionId((prev) => {
      if (prev && missionsForMap.some((mission) => mission.id === prev)) {
        return prev;
      }
      return missionsForMap[0]?.id ?? null;
    });
  }, [missionsForMap]);

  useEffect(() => {
    if (!selectedMap?.id) {
      setSelectedMissionId(null);
    }
  }, [selectedMap]);

  const missionDetails = selectedMissionId
    ? missionsForMap.find((mission) => mission.id === selectedMissionId)
    : null;

  const missionMapName = (mission: MissionRecord) => {
    const mapName = mapsList.find((map) => map.id === mission.mapId)?.name || mission.mapId;
    return mapName || "Unknown";
  };

  const handleSaveMission = async () => {
    if (!selectedMap?.id) {
      toast?.error?.("Select a map to create missions");
      return;
    }
    if (!missionForm.name.trim()) {
      toast?.error?.("Mission name required");
      return;
    }
    if (!missionForm.owner.trim()) {
      toast?.error?.("Mission owner required");
      return;
    }
    const payload: MissionRecord = {
      id: `mission-${Date.now()}`,
      mapId: selectedMap.id,
      name: missionForm.name.trim(),
      owner: missionForm.owner.trim(),
      status: missionForm.status,
      notes: missionForm.notes,
      createdAt: new Date().toISOString(),
    };

    try {
      let createdMission = payload;
      if (requestV1) {
        const response = await requestV1("/missions", {
          method: "POST",
          body: JSON.stringify(payload),
        });
        if (response?.item) {
          createdMission = { ...payload, ...response.item };
        }
      }
      setMissions((prev) => [createdMission, ...prev]);
      setSelectedMissionId(createdMission.id ?? null);
      setMissionForm(defaultFormState);
      setMissionFormOpen(false);
      toast?.success?.("Mission saved");
    } catch (error: any) {
      console.error("Mission create error", error);
      toast?.error?.(error?.message || "Failed to create mission");
    }
  };

  if (!selectedMap) {
    return (
      <section className="flex flex-col gap-4 px-4 py-6">
        <div className="rounded-2xl border border-slate-200 bg-white px-6 py-8 text-center text-slate-500">
          Select a map to view and manage missions.
        </div>
      </section>
    );
  }

  return (
    <section className="flex flex-col gap-4 px-4 py-4">
      <div className="flex flex-wrap items-center gap-3">
        <select
          value={missionSearchField}
          onChange={(event) => setMissionSearchField(event.target.value)}
          className="min-w-[160px] rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-600 outline-none transition focus:border-sky-500"
        >
          <option value="any">All Fields</option>
          <option value="name">Name</option>
          <option value="owner">Owner</option>
          <option value="status">Status</option>
          <option value="createdAt">Created At</option>
        </select>
        <input
          value={missionSearchTerm}
          onChange={(event) => setMissionSearchTerm(event.target.value)}
          placeholder="Search mission..."
          className="min-w-[220px] flex-1 rounded-xl border border-slate-200 px-4 py-2 text-sm text-slate-800 outline-none transition focus:border-sky-500"
        />
        <button
          type="button"
          className="inline-flex items-center gap-2 rounded-xl bg-[#0b74d1] px-4 py-3 text-sm font-semibold text-white shadow-lg shadow-sky-500/20 transition hover:bg-[#095da8] disabled:cursor-not-allowed disabled:opacity-60"
          onClick={() => setMissionFormOpen((previous) => !previous)}
        >
          <FaPlus className="text-xs" />
          <span>Create Mission</span>
        </button>
      </div>

      {missionFormOpen ? (
        <div className="rounded-2xl border border-slate-200 bg-slate-50/80 p-4">
          <div className="grid gap-4 md:grid-cols-2">
            <label className="flex flex-col gap-2 text-sm font-semibold text-slate-600">
              Name
              <input
                value={missionForm.name}
                onChange={(event) => setMissionForm((prev) => ({ ...prev, name: event.target.value }))}
                placeholder="Mission name"
                className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
              />
            </label>
            <label className="flex flex-col gap-2 text-sm font-semibold text-slate-600">
              Owner
              <input
                value={missionForm.owner}
                onChange={(event) => setMissionForm((prev) => ({ ...prev, owner: event.target.value }))}
                placeholder="Owner"
                className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
              />
            </label>
            <label className="flex flex-col gap-2 text-sm font-semibold text-slate-600">
              Status
              <select
                value={missionForm.status}
                onChange={(event) => setMissionForm((prev) => ({ ...prev, status: event.target.value }))}
                className="rounded-xl border border-slate-200 px-3 py-2 text-slate-800 focus:border-sky-500 focus:outline-none"
              >
                <option value="Draft">Draft</option>
                <option value="Scheduled">Scheduled</option>
                <option value="In Progress">In Progress</option>
                <option value="Completed">Completed</option>
              </select>
            </label>
          </div>
          <label className="mt-4 flex flex-col gap-2 text-sm font-semibold text-slate-600">
            Notes
            <textarea
              value={missionForm.notes}
              onChange={(event) => setMissionForm((prev) => ({ ...prev, notes: event.target.value }))}
              rows={3}
              className="rounded-xl border border-slate-200 px-3 py-2 text-sm text-slate-800 focus:border-sky-500 focus:outline-none"
            />
          </label>

          <div className="mt-4 flex justify-end gap-3">
            <button
              type="button"
              onClick={() => {
                setMissionForm(defaultFormState);
                setMissionFormOpen(false);
              }}
              className="rounded-xl border border-slate-200 px-4 py-2 text-sm font-semibold text-slate-600 transition hover:bg-white"
            >
              Cancel
            </button>
            <button
              type="button"
              onClick={handleSaveMission}
              className="rounded-xl bg-[#0b74d1] px-5 py-2 text-sm font-semibold text-white shadow-lg shadow-sky-500/20"
            >
              Save Mission
            </button>
          </div>
        </div>
      ) : null}

      <div className="overflow-hidden rounded-2xl border border-slate-200 bg-white">
        <table className="w-full border-collapse text-sm">
          <thead className="bg-slate-50 text-left text-xs font-semibold uppercase tracking-wide text-slate-500">
            <tr>
              <th className="px-4 py-3">Name</th>
              <th className="px-4 py-3">Owner</th>
              <th className="px-4 py-3">Status</th>
              <th className="px-4 py-3">Created</th>
            </tr>
          </thead>
          <tbody>
            {filteredMissions.length === 0 ? (
              <tr>
                <td colSpan={4} className="px-4 py-8 text-center text-sm text-slate-500">
                  {missionSearchTerm
                    ? `No missions match "${missionSearchTerm}"`
                    : `No missions found for ${selectedMap?.name}. Create one to get started.`}
                </td>
              </tr>
            ) : null}
            {filteredMissions.map((mission) => (
              <tr
                key={String(mission.id)}
                className={`cursor-pointer border-b border-slate-100 transition hover:bg-sky-50 ${
                  mission.id === selectedMissionId ? "bg-sky-50" : "bg-white"
                }`}
                onClick={() => setSelectedMissionId(mission.id ?? null)}
              >
                <td className="px-4 py-3 font-semibold text-slate-900">{mission.name}</td>
                <td className="px-4 py-3 text-slate-600">{mission.owner}</td>
                <td className="px-4 py-3">
                  <span className={`rounded-full px-3 py-1 text-xs font-semibold ${statusPillClasses(mission.status)}`}>
                    {mission.status}
                  </span>
                </td>
                <td className="px-4 py-3 text-slate-500">{mission.createdAt ? new Date(mission.createdAt).toLocaleString() : "—"}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      {missionDetails ? (
        <div className="rounded-2xl border border-slate-200 bg-slate-50/70 p-4">
          <div className="flex flex-col gap-2 text-sm text-slate-600">
            <div className="text-xs font-semibold uppercase tracking-widest text-slate-400">Mission Details</div>
            <div className="text-lg font-bold text-slate-900">{missionDetails.name}</div>
            <div>
              <span className="font-semibold text-slate-700">Owner:</span> {missionDetails.owner}
            </div>
            <div>
              <span className="font-semibold text-slate-700">Status:</span> {missionDetails.status}
            </div>
            <div>
              <span className="font-semibold text-slate-700">Map:</span> {missionMapName(missionDetails)}
            </div>
            <div>
              <span className="font-semibold text-slate-700">Created:</span> {missionDetails.createdAt ? new Date(missionDetails.createdAt).toLocaleString() : "—"}
            </div>
            {missionDetails.notes ? (
              <div>
                <span className="font-semibold text-slate-700">Notes:</span> {missionDetails.notes}
              </div>
            ) : null}
          </div>
        </div>
      ) : null}
    </section>
  );
};

export default MissionsPanel;

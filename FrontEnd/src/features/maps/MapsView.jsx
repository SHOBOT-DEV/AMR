import React, { useState } from "react";
import { FaPlus, FaEdit, FaTrash, FaCheckCircle } from "react-icons/fa";
import { toast } from "react-hot-toast";

const MapsView = ({ list, setList, selected, setSelected, requestV1 }) => {
  const [searchTerm, setSearchTerm] = useState("");
  const [filterField, setFilterField] = useState("name");

  // --- Logic extraction ---
  const handleActivate = async (map) => {
    // Optimistic Update
    const updatedList = list.map(m => ({
      ...m,
      status: m.id === map.id ? "Active" : ""
    }));
    setList(updatedList);
    setSelected(map);
    toast.success(`Activated: ${map.name}`);
    
    // API Call
    try {
      await requestV1(`/maps/${map.id}`, { method: "PATCH", body: JSON.stringify({ status: "Active" }) });
    } catch (e) {
      toast.error("Failed to sync activation");
    }
  };

  const filteredList = list.filter(m => {
    if (!searchTerm) return true;
    const val = m[filterField] ? String(m[filterField]).toLowerCase() : "";
    return val.includes(searchTerm.toLowerCase());
  });

  return (
    <div className="p-6 flex flex-col gap-6">
      {/* Actions Bar */}
      <div className="flex flex-col gap-4 bg-white p-4 rounded-xl border border-gray-200 shadow-sm">
        <div className="flex items-center justify-between gap-4">
          <div className="flex items-center gap-2 flex-1">
            <select 
              value={filterField} 
              onChange={(e) => setFilterField(e.target.value)}
              className="bg-gray-50 border border-gray-200 text-slate-700 text-sm rounded-lg focus:ring-blue-500 focus:border-blue-500 p-2.5 outline-none"
            >
              <option value="name">Name</option>
              <option value="createdBy">Created By</option>
              <option value="status">Status</option>
            </select>
            <input 
              type="text" 
              placeholder="Search maps..." 
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              className="bg-gray-50 border border-gray-200 text-slate-700 text-sm rounded-lg focus:ring-blue-500 focus:border-blue-500 block w-full p-2.5 outline-none"
            />
          </div>
          <button 
            className="flex items-center gap-2 text-white bg-blue-600 hover:bg-blue-700 font-medium rounded-lg text-sm px-5 py-2.5 transition-colors shadow-md shadow-blue-100"
            onClick={() => toast("Open Create Modal logic here")}
          >
            <FaPlus size={12} />
            <span>New Map</span>
          </button>
        </div>
      </div>

      {/* Table */}
      <div className="bg-white rounded-xl border border-gray-200 shadow-sm overflow-hidden">
        <table className="w-full text-sm text-left text-gray-500">
          <thead className="text-xs text-gray-700 uppercase bg-gray-50 border-b border-gray-100">
            <tr>
              <th className="px-6 py-4 w-16 text-center">Active</th>
              <th className="px-6 py-4">Name</th>
              <th className="px-6 py-4">Created By</th>
              <th className="px-6 py-4">Status</th>
              <th className="px-6 py-4 text-right">Actions</th>
            </tr>
          </thead>
          <tbody>
            {filteredList.map((map) => (
              <tr 
                key={map.id} 
                onClick={() => setSelected(map)}
                className={`border-b border-gray-50 hover:bg-gray-50 cursor-pointer transition-colors ${selected?.id === map.id ? "bg-blue-50/50" : ""}`}
              >
                <td className="px-6 py-4 text-center">
                  <div 
                    onClick={(e) => { e.stopPropagation(); handleActivate(map); }}
                    className={`w-5 h-5 rounded-full border-2 mx-auto cursor-pointer flex items-center justify-center transition-all ${map.status === "Active" ? "border-emerald-500 bg-emerald-500 text-white" : "border-gray-300 hover:border-blue-400"}`}
                  >
                    {map.status === "Active" && <FaCheckCircle size={10} />}
                  </div>
                </td>
                <td className="px-6 py-4 font-medium text-slate-900">{map.name}</td>
                <td className="px-6 py-4">{map.createdBy}</td>
                <td className="px-6 py-4">
                  <span className={`px-2.5 py-0.5 rounded-full text-xs font-medium ${map.status === "Active" ? "bg-emerald-100 text-emerald-800" : "bg-gray-100 text-gray-800"}`}>
                    {map.status || "Inactive"}
                  </span>
                </td>
                <td className="px-6 py-4 text-right">
                  <div className="flex items-center justify-end gap-2">
                    <button className="p-2 text-slate-400 hover:text-blue-600 transition-colors"><FaEdit /></button>
                    <button className="p-2 text-slate-400 hover:text-red-600 transition-colors"><FaTrash /></button>
                  </div>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default MapsView;
import React from "react";
import { FaDownload, FaTrash } from "react-icons/fa";

interface BagFile {
  id: string;
  name: string;
  duration: string;
  size: string;
  status: "Uploaded" | "Processing" | "Failed";
  createdAt?: string;
}

interface RobotBagsProps {
  bagFiles?: BagFile[];
}

const RobotBags: React.FC<RobotBagsProps> = ({
  bagFiles = [
    {
      id: "bag1",
      name: "mission-0915.bag",
      duration: "15m",
      size: "1.4 GB",
      status: "Uploaded",
      createdAt: "2025-12-29 09:15",
    },
    {
      id: "bag2",
      name: "mission-1030.bag",
      duration: "26m",
      size: "2.7 GB",
      status: "Processing",
      createdAt: "2025-12-29 10:30",
    },
    {
      id: "bag3",
      name: "mission-1145.bag",
      duration: "8m",
      size: "0.8 GB",
      status: "Failed",
      createdAt: "2025-12-29 11:45",
    },
  ],
}) => {
  const getStatusColor = (status: BagFile["status"]) => {
    switch (status) {
      case "Uploaded":
        return "bg-green-100 text-green-700";
      case "Processing":
        return "bg-blue-100 text-blue-700";
      case "Failed":
        return "bg-red-100 text-red-700";
      default:
        return "bg-slate-100 text-slate-700";
    }
  };

  return (
    <div className="flex flex-col gap-3">
      {bagFiles.map((bag) => (
        <div
          key={bag.id}
          className="bg-white border border-slate-200 rounded-lg p-4 hover:shadow-md transition-shadow"
        >
          <div className="flex justify-between items-start mb-2">
            <div>
              <strong className="text-sm font-semibold text-slate-900 block">
                {bag.name}
              </strong>
              {bag.createdAt && (
                <span className="text-xs text-slate-500">{bag.createdAt}</span>
              )}
            </div>
            <span
              className={`text-xs font-semibold px-2 py-1 rounded-full ${getStatusColor(
                bag.status
              )}`}
            >
              {bag.status}
            </span>
          </div>

          <div className="grid grid-cols-2 gap-2 my-3 text-xs text-slate-600">
            <p>Duration: {bag.duration}</p>
            <p>Size: {bag.size}</p>
          </div>

          <div className="flex gap-2">
            <button
              disabled={bag.status !== "Uploaded"}
              className="flex-1 flex items-center justify-center gap-2 px-3 py-2 bg-blue-50 hover:bg-blue-100 text-blue-700 font-medium rounded-lg text-sm transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
            >
              <FaDownload className="text-xs" />
              Download
            </button>
            <button className="flex-1 flex items-center justify-center gap-2 px-3 py-2 bg-red-50 hover:bg-red-100 text-red-700 font-medium rounded-lg text-sm transition-colors">
              <FaTrash className="text-xs" />
              Delete
            </button>
          </div>
        </div>
      ))}
    </div>
  );
};

export default RobotBags;

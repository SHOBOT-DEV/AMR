import React from "react";

interface CameraProps {
  cameraStreams?: Array<{
    id: string;
    name: string;
    status: "Active" | "Inactive" | "Error";
    resolution?: string;
    fps?: number;
  }>;
}

const Camera: React.FC<CameraProps> = ({
  cameraStreams = [
    {
      id: "cam1",
      name: "Front Camera",
      status: "Active",
      resolution: "1920x1080",
      fps: 30,
    },
    {
      id: "cam2",
      name: "Rear Camera",
      status: "Active",
      resolution: "1280x720",
      fps: 24,
    },
    {
      id: "cam3",
      name: "Thermal Camera",
      status: "Inactive",
      resolution: "640x480",
      fps: 15,
    },
  ],
}) => {
  const getStatusColor = (status: string) => {
    switch (status) {
      case "Active":
        return "bg-green-100 text-green-700";
      case "Inactive":
        return "bg-gray-100 text-gray-700";
      case "Error":
        return "bg-red-100 text-red-700";
      default:
        return "bg-slate-100 text-slate-700";
    }
  };

  return (
    <div className="flex flex-col gap-4">
      <div className="bg-slate-900 rounded-lg aspect-video flex items-center justify-center">
        <div className="text-center">
          <div className="text-slate-400 text-sm mb-2">Camera Feed</div>
          <div className="text-slate-500 text-xs">Select a camera to view stream</div>
        </div>
      </div>

      <div className="flex flex-col gap-2">
        {cameraStreams.map((stream) => (
          <div
            key={stream.id}
            className="bg-white border border-slate-200 rounded-lg p-3 hover:shadow-md transition-shadow cursor-pointer"
          >
            <div className="flex justify-between items-start mb-2">
              <strong className="text-sm font-semibold text-slate-900">
                {stream.name}
              </strong>
              <span className={`text-xs font-semibold px-2 py-1 rounded-full ${getStatusColor(stream.status)}`}>
                {stream.status}
              </span>
            </div>
            <div className="text-xs text-slate-600 space-y-1">
              <p>Resolution: {stream.resolution}</p>
              <p>FPS: {stream.fps}</p>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default Camera;

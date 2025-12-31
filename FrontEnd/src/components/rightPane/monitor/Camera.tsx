import React, { useState } from "react";
import { MdFlipCameraAndroid } from "react-icons/md";

interface CameraStream {
  id: string;
  name: string;
  status: "Active" | "Inactive" | "Error";
  resolution?: string;
  fps?: number;
}

interface CameraProps {
  cameraStreams?: CameraStream[];
}

const Camera: React.FC<CameraProps> = ({
  cameraStreams = [
    {
      id: "front",
      name: "Front Camera",
      status: "Active",
      resolution: "1920x1080",
      fps: 30,
    },
    {
      id: "rear",
      name: "Rear Camera",
      status: "Inactive",
      resolution: "1280x720",
      fps: 24,
    },
    {
      id: "thermal",
      name: "Thermal Camera",
      status: "Inactive",
      resolution: "640x480",
      fps: 15,
    },
  ],
}) => {
  const [streams, setStreams] = useState<CameraStream[]>(cameraStreams);

  const activeCamera = streams.find((c) => c.status === "Active");

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

  // 🔁 Activate only selected camera
  const activateCamera = (id: string) => {
    setStreams((prev) =>
      prev.map((cam) => ({
        ...cam,
        status: cam.id === id ? "Active" : "Inactive",
      }))
    );
  };

  // 📱 Flip between front ↔ rear camera
  const flipCamera = () => {
    const front = streams.find((c) => c.id === "front");
    const rear = streams.find((c) => c.id === "rear");

    if (!front || !rear) return;

    activateCamera(front.status === "Active" ? "rear" : "front");
  };

  return (
    <div className="flex flex-col gap-4">
      {/* 🎥 Camera Feed */}
      <div className="relative bg-slate-900 rounded-lg aspect-video flex items-center justify-center">
        {/* 🔄 Flip camera icon */}
        <button
          onClick={flipCamera}
          title="Flip camera"
          className="absolute top-2 right-2 text-white bg-slate-800/70 hover:bg-slate-700 p-2 rounded-full transition"
        >
          <MdFlipCameraAndroid size={20} />
        </button>

        {/* 📺 Active camera info */}
        <div className="text-center">
          <div className="text-slate-300 text-sm mb-1">
            {activeCamera?.name || "Camera Feed"}
          </div>
          <div className="text-slate-500 text-xs">
            {activeCamera ? "Live stream active" : "Select a camera"}
          </div>
        </div>
      </div>

      {/* 📋 Camera list */}
      <div className="flex flex-col gap-2">
        {streams.map((stream) => (
          <div
            key={stream.id}
            onClick={() => activateCamera(stream.id)}
            className="bg-white border border-slate-200 rounded-lg p-3 hover:shadow-md transition-shadow cursor-pointer"
          >
            <div className="flex justify-between items-start mb-2">
              <strong className="text-sm font-semibold text-slate-900">
                {stream.name}
              </strong>
              <span
                className={`text-xs font-semibold px-2 py-1 rounded-full ${getStatusColor(
                  stream.status
                )}`}
              >
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

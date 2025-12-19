import React, { useState } from "react";
import JoyStick from "./JoyStick"; // Keep your existing component

const MapArea = ({ minimized, onJoystickMove }) => {
  const [zoomLevel, setZoomLevel] = useState(1);

  const handleZoom = (delta) => {
    setZoomLevel(prev => Math.min(Math.max(prev + delta, 0.5), 3));
  };

  return (
    <main className={`relative transition-all duration-300 ease-in-out bg-slate-100 overflow-hidden flex items-center justify-center ${minimized ? "w-[60%]" : "w-full"}`}>
      
      {/* Map Content Container */}
      <div 
        className="w-full h-full flex items-center justify-center transition-transform duration-200"
        style={{ transform: `scale(${zoomLevel})` }}
      >
        {/* Placeholder for actual Map Canvas/Image */}
        <div className="w-[800px] h-[600px] border-2 border-dashed border-slate-300 rounded-xl bg-white/50 flex items-center justify-center">
          <span className="text-slate-400 font-medium">Map Visualization Area</span>
        </div>
      </div>

      {/* Overlays */}
      <div className="absolute bottom-6 right-6 flex flex-col gap-4 items-end">
        {/* Zoom Controls */}
        <div className="flex flex-col bg-white rounded-lg shadow-md border border-gray-200 overflow-hidden">
          <button onClick={() => handleZoom(0.1)} className="p-3 hover:bg-gray-50 active:bg-gray-100 border-b border-gray-100 text-slate-600 font-bold">+</button>
          <button onClick={() => handleZoom(-0.1)} className="p-3 hover:bg-gray-50 active:bg-gray-100 text-slate-600 font-bold">−</button>
        </div>

        {/* Joystick */}
        <div className={`bg-white/80 backdrop-blur-sm p-4 rounded-2xl shadow-lg border border-white/50 transition-opacity ${minimized ? "opacity-0 pointer-events-none" : "opacity-100"}`}>
          <JoyStick width={120} height={120} onMove={onJoystickMove} />
        </div>
      </div>
    </main>
  );
};

export default MapArea;
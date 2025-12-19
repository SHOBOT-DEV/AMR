import React from "react";

// Formerly "RightPanel"
const FloatingMenu = ({ pos, children }) => (
  <div
    style={{ left: pos.left, top: pos.top }}
    className="
      fixed z-[90] min-w-[200px] p-2 flex flex-col gap-1
      rounded-r-xl rounded-bl-xl
      bg-gradient-to-b from-sky-100 via-sky-200 to-sky-300
      shadow-[4px_8px_24px_rgba(2,12,22,0.18)]
      border border-white/40 backdrop-blur-sm
      animate-in fade-in zoom-in-95 duration-150
    "
  >
    {children}
  </div>
);

export const MenuItem = ({ icon: Icon, label, onClick }) => (
  <button
    onClick={(e) => {
      e.stopPropagation();
      e.preventDefault();
      if (onClick) onClick();
    }}
    className="
      group flex items-center gap-3 px-3 py-2.5
      rounded-lg font-semibold text-sm text-slate-800
      hover:bg-white/40 hover:text-sky-900
      transition-all duration-200
    "
  >
    {Icon && <Icon className="text-sky-700 group-hover:text-sky-900 text-lg" />}
    <span>{label}</span>
  </button>
);

export default FloatingMenu;
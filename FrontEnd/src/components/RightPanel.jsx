import React from "react";

interface RightPanelProps {
  pos: { left: number; top: number };
  children: React.ReactNode;
}

const RightPanel: React.FC<RightPanelProps> = ({ pos, children }) => (
  <div
    style={{ left: pos.left, top: pos.top }}
    className="
      fixed z-[90]
      w-[220px]
      p-3
      flex flex-col gap-2
      rounded-r-xl
      bg-gradient-to-b from-sky-200 via-sky-300 to-sky-400
      shadow-[4px_8px_24px_rgba(2,12,22,0.18)]
      border-l border-white/20
    "
  >
    {children}
  </div>
);

interface PanelItemProps {
  icon: React.ComponentType<{ className?: string }>;
  label: string;
  onClick?: () => void;
}

export const PanelItem: React.FC<PanelItemProps> = ({
  icon: Icon,
  label,
  onClick,
}) => (
  <button
    onClick={onClick}
    className="
      group flex items-center gap-3
      px-3 py-2
      rounded-lg font-semibold
      text-slate-900
      hover:bg-white/20 hover:text-white
      transition
    "
  >
    <Icon className="text-sky-800 group-hover:text-white" />
    <span>{label}</span>
  </button>
);

export default RightPanel;

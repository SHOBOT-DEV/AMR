// UI
{/* APPEARANCE */}
{rightPage === "appearance" && (
  <div className="appearance-pane">
    {[
      { id: "light", label: "Light" },
      { id: "dark", label: "Dark" },
      { id: "system", label: "Match System" },
    ].map((theme) => (
      <button
        key={theme.id}
        className={`theme-card ${selectedTheme === theme.id ? "active" : ""}`}
        onClick={() => setSelectedTheme(theme.id)}
      >
        <strong>{theme.label}</strong>
        <span>{selectedTheme === theme.id ? "Selected" : "Use theme"}</span>
      </button>
    ))}
  </div>
)}
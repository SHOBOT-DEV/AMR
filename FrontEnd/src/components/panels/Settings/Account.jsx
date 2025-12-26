    // UI
        {/* ACCOUNT */}
        {rightPage === "account" && (
          <div className="account-pane">
            <label>
              Name
              <input value={accountProfile.fullName} onChange={(e) => handleAccountChange("fullName", e.target.value)} />
            </label>
            <label>
              Email
              <input value={accountProfile.email} onChange={(e) => handleAccountChange("email", e.target.value)} />
            </label>
            <label>
              Team
              <input value={accountProfile.team} onChange={(e) => handleAccountChange("team", e.target.value)} />
            </label>
            <label>
              Shift Window
              <input value={accountProfile.shift} onChange={(e) => handleAccountChange("shift", e.target.value)} />
            </label>
            {profileError && <p style={{ color: "red" }}>{profileError}</p>}
            {profileSuccess && <p style={{ color: "green" }}>{profileSuccess}</p>}
            <button className="primary-btn" type="button" onClick={handleSaveProfile} disabled={profileSaving}>{profileSaving ? "Saving..." : "Save Profile"}</button>
          </div>
        )}
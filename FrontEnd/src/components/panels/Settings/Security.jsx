// UI

        {/* SECURITY */}
        {rightPage === "security" && (
          <div className="security-pane">
            <div className="security-toggles">
              {Object.entries(securityPreferences).map(([key, value]) => (
                <label key={key} className="toggle-row">
                  <input type="checkbox" checked={value} onChange={() => toggleSecurityPref(key)} />
                  <div>
                    <strong>{key.replace(/([A-Z])/g, " $1")}</strong>
                    <p>{value ? "Enabled" : "Disabled"}</p>
                  </div>
                </label>
              ))}
            </div>
            <table className="security-table">
              <thead>
                <tr>
                  <th>Time</th>
                  <th>Actor</th>
                  <th>Action</th>
                  <th>Context</th>
                </tr>
              </thead>
              <tbody>
                {securityEvents.map((evt) => (
                  <tr key={evt.id}>
                    <td>{evt.ts}</td>
                    <td>{evt.actor}</td>
                    <td>{evt.action}</td>
                    <td>{evt.context}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}
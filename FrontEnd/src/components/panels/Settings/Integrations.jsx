// UI

        {/* INTEGRATIONS */}
        {rightPage === "integrations" && (
          <div className="integrations-pane">
            {integrationItems.map((integration) => (
              <div key={integration.id} className="integration-card">
                <div>
                  <strong>{integration.name}</strong>
                  <p>{integration.description}</p>
                </div>
                <div className="integration-meta">
                  <span className={`integration-status ${integration.status === "Connected" ? "ok" : "off"}`}>{integration.status}</span>
                  <button className="ghost-btn" onClick={() => toggleIntegrationStatus && toggleIntegrationStatus(integration.id)}>{integration.status === "Connected" ? "Disconnect" : "Connect"}</button>
                </div>
              </div>
            ))}
          </div>
        )}
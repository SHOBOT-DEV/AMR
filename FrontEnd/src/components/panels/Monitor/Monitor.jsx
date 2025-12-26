    const loadMonitorData = async () => {
      try {
        const [analyticsRes, diagnosticsRes, logsRes, missionLogsRes, bagsRes] =
          await Promise.all([
            requestV1("/monitor/analytics"),
            requestV1("/monitor/diagnostics"),
            requestV1("/monitor/logs"),
            requestV1("/monitor/mission-logs"),
            requestV1("/monitor/robot-bags"),
          ]);

        if (cancelled) {
          return;
        }

        const analyticsPayload = analyticsRes.data || {};
        if (Array.isArray(analyticsPayload.summary)) {
          setAnalyticsSummary(analyticsPayload.summary);
        }
        if (Array.isArray(analyticsPayload.series)) {
          setAnalyticsSeries(analyticsPayload.series);
        }
        if (Array.isArray(analyticsPayload.alerts)) {
          setAnalyticsAlerts(analyticsPayload.alerts);
        }
        if (Array.isArray(diagnosticsRes.items)) {
          setDiagnosticsPanels(diagnosticsRes.items);
        }
        if (Array.isArray(logsRes.items)) {
          setLogEvents(logsRes.items);
        }
        if (Array.isArray(missionLogsRes.items)) {
          setMissionHistory(missionLogsRes.items);
        }
        if (Array.isArray(bagsRes.items)) {
          setBagFiles(bagsRes.items);
        }
      } catch (error) {
        console.error("Failed to load monitor data", error);
      }
    };

    loadMonitorData();
    const intervalId = setInterval(loadMonitorData, 60000);
    return () => {
      cancelled = true;
      clearInterval(intervalId);
    };
  }, [requestV1]);

  useEffect(() => {
    let cancelled = false;
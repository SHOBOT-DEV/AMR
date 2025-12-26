//  logic

import { useCallback } from "react";

import { useNavigate } from "react-router-dom";

 const handleLogout = useCallback(() => {
    // clear tokens and go back to login
    clearAuthTokens();
    navigate("/");
  }, [navigate]);

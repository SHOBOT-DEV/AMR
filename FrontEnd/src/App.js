import { BrowserRouter as Router, Routes, Route, Navigate, useNavigate } from "react-router-dom";
import Login from "./components/login";
import Register from "./components/Register";
import AdminLogin from "./components/AdminLogin";
import Admin from "./components/Admin";  // Admin dashboard page
import MainPage from "./components/MainPage";

function AppRoutes() {
  const navigate = useNavigate();

  return (
    <Routes>
      {/* Public user login */}
      <Route
        path="/"
        element={
          <Login
            onLogin={() => navigate("/main")}        // public login -> main page
            onSignUpClick={() => navigate("/register")}
          />
        }
      />

      {/* Register */}
      <Route
        path="/register"
        element={<Register onRegister={() => navigate("/")} onLoginClick={() => navigate("/")} />}
      />

      {/* Admin login page */}
      <Route path="/admin"
      element={<AdminLogin />}
      />

      {/* Admin dashboard (requires correct admin credentials in AdminLogin) */}
      <Route path="/admin/dashboard"
      element={<Admin />}
      />

      {/* Main (post-login) page */}
      <Route path="/main" element={<MainPage />}
      />

      {/* Catch-all for unknown routes */}
      <Route path="*" element={<Navigate to="/" replace />}
      />
    </Routes>
  );
}

export default function App() {
  return (
    <Router>
      <AppRoutes />
    </Router>
  );
}

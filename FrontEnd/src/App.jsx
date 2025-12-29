import {
  BrowserRouter as Router,
  Routes,
  Route,
  Navigate,
  useNavigate,
} from "react-router-dom";
import { Toaster } from "react-hot-toast";
import MainPage from "./components/layout/DashboardLayout.tsx";
import Admin from "./pages/admin/Admin.tsx"; 
import AdminLogin from "./pages/auth/AdminLogin.tsx";
import Login from "./pages/auth/login.tsx";
import Register from "./pages/auth/Register.tsx";

function AppRoutes() {
  const navigate = useNavigate();

  return (
    <Routes>
      {/* Public user login */}
      <Route
        path="/"
        element={
          <Login
            onLogin={() => navigate("/main")} // public login -> main page
            onSignUpClick={() => navigate("/register")}
          />
        }
      />

      {/* Register */}
      <Route
        path="/register"
        element={
          <Register
            onRegister={() => navigate("/")}
            onLoginClick={() => navigate("/")}
          />
        }
      />

      {/* Admin login page */}
      <Route path="/admin" element={<AdminLogin />} />

      {/* Admin dashboard (requires correct admin credentials in AdminLogin) */}
      <Route path="/admin/dashboard" element={<Admin />} />

      {/* Main (post-login) page */}
      <Route path="/main" element={<MainPage />} />

      {/* Catch-all for unknown routes */}
      <Route path="*" element={<Navigate to="/" replace />} />
    </Routes>
  );
}

export default function App() {
  return (
    <Router>
      <AppRoutes />
      <Toaster position="top-right" />
    </Router>
  );
}

import React, { useState, FormEvent } from "react";
import { useNavigate } from "react-router-dom";
import LeftImage from "../assets/Shobot_WBg.png";
import IITLogo from "../assets/IIT_Logo.png";
import { storeAuthTokens, API_BASE } from "../utils/auth";

const API_URL = API_BASE || "http://127.0.0.1:5000";

const AdminLogin: React.FC = () => {
  const [username, setUsername] = useState<string>("");
  const [password, setPassword] = useState<string>("");
  const [error, setError] = useState<string>("");
  const navigate = useNavigate();

  const handleSubmit = async (e: FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    setError("");

    try {
      const response = await fetch(`${API_URL}/api/login`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          username: username.trim(),
          password: password.trim(),
        }),
      });

      const data = (await response.json().catch(() => ({}))) as any;

      if (response.ok && data?.success) {
        if (data.user?.role === "admin") {
          storeAuthTokens({
            token: data.token,
            refreshToken: data.refreshToken,
            isAdmin: true,
          });
          setError("");
          navigate("/admin/dashboard");
        } else {
          setError("Admin access required. This account is not an admin.");
        }
      } else {
        setError(data?.message || "Invalid credentials, please try again.");
      }
    } catch (err: any) {
      console.error(err);
      setError("Network error. Make sure Flask server is running on port 5000.");
    }
  };

  return (
    <div className="min-h-screen flex">
      {/* Left: image for md+ screens */}
      <div
        className="hidden md:block md:w-1/2 bg-cover bg-center"
        style={{ backgroundImage: `url(${LeftImage})` }}
        aria-hidden="true"
      />

      {/* Right: login form */}
      <div className="w-full md:w-1/2 flex items-center justify-center p-8 bg-gray-50">
        <div className="w-full max-w-md bg-white rounded-lg shadow-md p-8">
          <div className="flex flex-col items-center">
            <img src={IITLogo} alt="IIT Logo" className="h-12 mb-4" />
            <h1 className="text-2xl font-semibold text-gray-800 mb-2">
              Admin Login
            </h1>
            {error && (
              <p className="text-sm text-red-600 mb-4 text-center">{error}</p>
            )}
          </div>

          <form onSubmit={handleSubmit} className="space-y-4">
            <input
              type="text"
              placeholder="Username"
              value={username}
              onChange={(e) => setUsername(e.target.value)}
              required
              className="w-full px-4 py-2 border rounded focus:outline-none focus:ring-2 focus:ring-indigo-200"
            />

            <input
              type="password"
              placeholder="Password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
              className="w-full px-4 py-2 border rounded focus:outline-none focus:ring-2 focus:ring-indigo-200"
            />

            <button
              type="submit"
              className="w-full py-2 bg-indigo-600 text-white rounded hover:bg-indigo-700 transition"
            >
              Admin Login
            </button>
          </form>
        </div>
      </div>
    </div>
  );
};

export default AdminLogin;

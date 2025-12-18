import { useState, type FormEvent } from "react";
import LeftImage from "../assets/Shobot_Img.png";
import IITLogo from "../assets/IIT_Logo.png";
import { storeAuthTokens, API_BASE } from "../utils/auth";

interface LoginProps {
  onLogin: () => void;
  onSignUpClick: () => void;
}

const Login = ({ onLogin, onSignUpClick }: LoginProps) => {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");

  const handleSubmit = async (e: FormEvent<HTMLFormElement>) => {
    e.preventDefault();

    try {
      const response = await fetch(`${API_BASE}/api/login`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          username: username.trim(),
          password: password.trim(),
        }),
      });

      const data = await response.json();

      if (response.ok && data.success) {
        setError("");
        storeAuthTokens({
          token: data.token,
          refreshToken: data.refreshToken,
        });
        onLogin();
      } else {
        setError(data.message || "Login failed. Please try again.");
      }
    } catch {
      setError("Network error. Make sure Flask server is running on port 5000.");
    }
  };

  return (
    <div className="flex h-screen w-full overflow-hidden">
      {/* Left image */}
      <div
        className="hidden md:block flex-1 bg-cover bg-center"
        style={{ backgroundImage: `url(${LeftImage})` }}
      />

      {/* Right side */}
      <div className="flex flex-1 items-center justify-center bg-gradient-to-br from-cyan-200 to-blue-400 p-10">
        <div className="w-full max-w-md rounded-xl bg-white p-10 shadow-xl text-center">
          <img
            src={IITLogo}
            alt="IIT logo"
            className="mx-auto mb-4 w-20"
          />

          <h1 className="mb-6 text-2xl font-semibold text-gray-800">
            Login
          </h1>

          {error && <p className="mb-3 text-red-500">{error}</p>}

          <form onSubmit={handleSubmit} className="space-y-4">
            <input
              className="w-full rounded-md border border-gray-300 p-2 text-base"
              type="text"
              placeholder="Username"
              value={username}
              onChange={(e) => setUsername(e.target.value)}
              required
            />

            <input
              className="w-full rounded-md border border-gray-300 p-2 text-base"
              type="password"
              placeholder="Password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
            />

            <button
              type="submit"
              className="w-full rounded-md bg-blue-600 py-2 text-white transition hover:bg-blue-700"
            >
              Login
            </button>
          </form>

          <span
            onClick={onSignUpClick}
            className="mt-6 block cursor-pointer font-medium text-blue-600 hover:underline"
          >
            Sign up
          </span>
        </div>
      </div>
    </div>
  );
};

export default Login;

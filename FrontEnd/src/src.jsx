

import { useState } from "react";
import Register from "./components/Register";
import Login from "./components/login";
import MainPage from "./components/MainPage";

function App() {
  const [page, setPage] = useState("login");

  // Simulate successful login
  const handleLogin = () => {
    setPage("main");
  };

  // Handle registration success and data
  const handleRegister = (userData) => {
    console.log("User Data:", userData); // You can now use the user data for API calls or state
    setPage("login"); // Go to login page after registration
  };

  return (
    <div className="App">
      {page === "login" && <Login onLogin={handleLogin} onSignUpClick={() => setPage("register")} />}
      {page === "register" && <Register onRegister={handleRegister} onLoginClick={() => setPage("login")} />}
      {page === "main" && <MainPage />}
    </div>
  );
}

export default App;



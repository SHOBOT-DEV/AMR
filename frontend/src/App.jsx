import { useState } from "react";
import Login from "pages/login";
import "styles/App.css";

function App() {
  const [isLoggedIn, setIsLoggedIn] = useState(false);

  const handleLogin = () => {
    setIsLoggedIn(true);
  };

  const handleSignUpClick = () => {
    // Handle sign up navigation
    console.log("Sign up clicked");
  };

  if (!isLoggedIn) {
    return <Login onLogin={handleLogin} onSignUpClick={handleSignUpClick} />;
  }

  return (
    <div>
      <h1>Welcome!</h1>
      <button onClick={() => setIsLoggedIn(false)}>Logout</button>
    </div>
  );
}

export default App;

// Vite environment variable
export const API_BASE =
  import.meta.env.VITE_API_BASE || "http://127.0.0.1:5000";

/**
 * Store auth tokens in localStorage
 */
export const storeAuthTokens = ({ token, refreshToken }) => {
  if (token) localStorage.setItem("access_token", token);
  if (refreshToken) localStorage.setItem("refresh_token", refreshToken);
};

/**
 * Get stored access token
 */
export const getAccessToken = () => {
  return localStorage.getItem("access_token");
};

/**
 * Get stored refresh token
 */
export const getRefreshToken = () => {
  return localStorage.getItem("refresh_token");
};

/**
 * Clear auth data (logout)
 */
export const clearAuthTokens = () => {
  localStorage.removeItem("access_token");
  localStorage.removeItem("refresh_token");
};

// const API_BASE_URL = process.env.REACT_APP_API_URL || "http://127.0.0.1:5000";
// const REFRESH_URL = `${API_BASE_URL}/api/refresh`;

// export const API_BASE = import.meta.env.VITE_API_BASE || "http://localhost:5173";

// export const storeAuthTokens = ({
//   token,
//   refreshToken,
//   isAdmin = false,
// } = {}) => {
//   if (token) {
//     localStorage.setItem("token", token);
//     if (isAdmin) {
//       localStorage.setItem("adminToken", token);
//     }
//   }
//   if (refreshToken) {
//     localStorage.setItem("refreshToken", refreshToken);
//   }
// };

// export const getAuthToken = () => {
//   return localStorage.getItem("authToken");
// };

// export const clearAuthTokens = () => {
//   localStorage.removeItem("token");
//   localStorage.removeItem("adminToken");
//   localStorage.removeItem("refreshToken");
// };

// const getStoredAccessToken = () =>
//   localStorage.getItem("adminToken") || localStorage.getItem("token");

// export const refreshAccessToken = async () => {
//   const refreshToken = localStorage.getItem("refreshToken");
//   if (!refreshToken) {
//     return null;
//   }

//   try {
//     const response = await fetch(REFRESH_URL, {
//       method: "POST",
//       headers: {
//         "Content-Type": "application/json",
//       },
//       body: JSON.stringify({ refreshToken }),
//     });

//     const data = await response.json().catch(() => ({}));
//     if (!response.ok || !data.success) {
//       clearAuthTokens();
//       return null;
//     }

//     storeAuthTokens({
//       token: data.token,
//       refreshToken: data.refreshToken,
//       isAdmin: Boolean(localStorage.getItem("adminToken")),
//     });

//     return data.token;
//   } catch (error) {
//     console.error("Failed to refresh access token", error);
//     return null;
//   }
// };

// export const fetchWithAuth = async (url, options = {}) => {
//   let token = getStoredAccessToken();
//   const headers = {
//     ...(options.headers || {}),
//   };

//   if (!token) {
//     throw new Error("No access token available");
//   }

//   headers.Authorization = `Bearer ${token}`;

//   let response = await fetch(url, { ...options, headers });

//   if (response.status === 401) {
//     token = await refreshAccessToken();
//     if (!token) {
//       return response;
//     }
//     headers.Authorization = `Bearer ${token}`;
//     response = await fetch(url, { ...options, headers });
//   }

//   return response;
// };

## AMR Control Platform

A two-part application for managing autonomous mobile robots (AMRs). The Flask backend exposes authentication, administration, and telemetry APIs, while the React frontend provides operator and admin dashboards (tele-operator console, approvals, analytics, etc.).

### Project layout

```
AMR/
├── backend/          # Flask app (API, models, routes, config)
├── FrontEnd/         # React single-page application
├── UI/               # Legacy playground components (not wired into main app)
├── run.sh            # Helper script to start backend/frontend together
└── README.md
```

### Requirements

- Python 3.12+
- Node.js 18+ / npm 10+
- SQLite (bundled with Python)

### Backend setup

```bash
cd backend
python -m venv venv
source venv/bin/activate      # On Windows use: venv\Scripts\activate
pip install -r requirements.txt
python run.py                 # Starts Flask on http://127.0.0.1:5000
```

> The first run creates `backend/instance/users.db` plus a default admin user (`admin` / `admin123`).

Key env vars (see `backend/config.py` for defaults):

- `SECRET_KEY` – JWT signing key
- `ACCESS_TOKEN_EXPIRES_HOURS` – Access token lifetime (default 24h)
- `REFRESH_TOKEN_EXPIRES_DAYS` – Refresh token lifetime (default 30d)

### Frontend setup

```bash
cd FrontEnd
npm install
npm start                   # Runs on http://localhost:3000 with proxy -> Flask
```

Set `REACT_APP_API_URL` to override the API base (defaults to `http://127.0.0.1:5000`).

### Authentication flow

1. `/api/login` issues a short-lived JWT **and** a long-lived refresh token.
2. Refresh tokens persist in `refresh_tokens` (see `backend/models.py`) and rotate on each login/refresh.
3. The frontend stores the pair via `src/utils/auth.js` and uses `fetchWithAuth` to automatically refresh expired access tokens before retrying protected requests.
4. `/api/refresh` exchanges a valid refresh token for a new pair.

### Available scripts

- **Backend**
  - `python backend/run.py` – Development server
- **Frontend**
  - `npm start` / `npm run build` – CRA scripts
  - `npx prettier --write "src/**/*.{js,jsx,css}"` – Format UI code
- **Formatting**
  - `python -m black backend` – Format Python modules

### Running both tiers

Use separate terminals (backend on :5000, frontend on :3000) or tweak `run.sh` with a process manager such as `honcho`/`foreman`.

### Testing

- Backend: add `pytest` or Flask testing setup (not included yet).
- Frontend: `npm test` (React Testing Library scaffolding exists).

### Notes

- Admin dashboard lives at `/admin` and requires an approved admin account.
- Main tele-operator dashboard is served at `/` post-login.
- Profile changes in the “Account” pane call `PUT /api/user/profile`.
- Emergency stop button toggles in the header and surfaces feedback via toast notifications.

Feel free to extend this README with deployment steps, CI documentation, or API references as the project grows.

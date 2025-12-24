import React, { useMemo, useState } from "react";
import { FaEdit } from "react-icons/fa";

type UserRecord = {
  id: string | number;
  username: string;
  email: string;
  company?: string;
  amr_type?: string;
  role?: string;
  approval?: string;
};

type ToastApi = {
  success?: (message: string) => void;
  error?: (message: string) => void;
};

type Props = {
  users: UserRecord[];
  setUsers: React.Dispatch<React.SetStateAction<UserRecord[]>>;
  usersLoading: boolean;
  usersError?: string;
  loadUsers: () => void | Promise<void>;
  selectedUserId: string | number | null;
  setSelectedUserId: (value: string | number | null) => void;
  userActionLoading: boolean;
  handleResetUserPassword: () => void;
  toast?: ToastApi;
};

const fieldMap = {
  username: "username",
  email: "email",
  company: "company",
  amr_type: "amr_type",
  role: "role",
  approval: "approval",
} as const;

const blankForm = {
  username: "",
  email: "",
  company: "",
  amr_type: "",
  role: "user",
  approval: "Pending",
};

const UsersPanel: React.FC<Props> = ({
  users,
  setUsers,
  usersLoading,
  usersError,
  loadUsers,
  selectedUserId,
  setSelectedUserId,
  userActionLoading,
  handleResetUserPassword,
  toast,
}) => {
  const [userSearchField, setUserSearchField] = useState<keyof typeof fieldMap | "any">("any");
  const [userSearchTerm, setUserSearchTerm] = useState("");
  const [editingUserId, setEditingUserId] = useState<string | number | null>(null);
  const [editUserForm, setEditUserForm] = useState(blankForm);

  const filteredUsers = useMemo(() => {
    const term = userSearchTerm.trim().toLowerCase();
    if (!term) return users;
    if (userSearchField === "any") {
      return users.filter((user) =>
        Object.values(fieldMap)
          .map((field) => String(user[field as keyof UserRecord] || "").toLowerCase())
          .some((value) => value.includes(term)),
      );
    }
    const fieldName = fieldMap[userSearchField];
    return users.filter((user) => String(user[fieldName as keyof UserRecord] || "").toLowerCase().includes(term));
  }, [userSearchField, userSearchTerm, users]);

  const selectedUser = selectedUserId ? users.find((user) => user.id === selectedUserId) : null;
  const editingUser = editingUserId ? users.find((user) => user.id === editingUserId) : null;

  const startEditing = (user: UserRecord) => {
    setEditingUserId(user.id);
    setEditUserForm({
      username: user.username || "",
      email: user.email || "",
      company: user.company || "",
      amr_type: user.amr_type || "",
      role: user.role || "user",
      approval: user.approval || "Pending",
    });
  };

  const cancelEditing = () => {
    setEditingUserId(null);
    setEditUserForm(blankForm);
  };

  const saveUserChanges = () => {
    if (!editingUser) return;
    setUsers((previous) =>
      previous.map((user) => (user.id === editingUser.id ? { ...user, ...editUserForm } : user)),
    );
    toast?.success?.("User updated successfully");
    cancelEditing();
  };

  return (
    <section className="flex flex-col gap-4 px-4 py-4">
      {usersError ? (
        <div className="rounded-2xl border border-red-200 bg-red-50 px-4 py-3 text-sm text-red-700">{usersError}</div>
      ) : null}

      <div className="flex flex-wrap items-center gap-3">
        <select
          value={userSearchField}
          onChange={(event) => setUserSearchField(event.target.value as keyof typeof fieldMap | "any")}
          className="min-w-[160px] rounded-xl border border-slate-200 bg-white px-3 py-2 text-sm text-slate-600 outline-none transition focus:border-sky-500"
        >
          <option value="any">All Fields</option>
          {Object.keys(fieldMap).map((key) => (
            <option key={key} value={key}>
              {key.replace("_", " ").replace(/\b\w/g, (char) => char.toUpperCase())}
            </option>
          ))}
        </select>
        <input
          value={userSearchTerm}
          onChange={(event) => setUserSearchTerm(event.target.value)}
          placeholder="Search user..."
          className="min-w-[220px] flex-1 rounded-xl border border-slate-200 px-4 py-2 text-sm text-slate-800 outline-none transition focus:border-sky-500"
        />
        <button
          type="button"
          onClick={() => loadUsers()}
          className="rounded-xl border border-slate-200 px-4 py-2 text-sm font-semibold text-slate-700 transition hover:border-slate-300 hover:bg-white"
        >
          Refresh
        </button>
      </div>

      {editingUser ? (
        <div className="rounded-2xl border border-sky-200 bg-sky-50/80 p-4">
          <div className="mb-3 flex items-center justify-between">
            <div className="text-base font-semibold text-slate-800">Edit User: {editingUser.username}</div>
            <button
              type="button"
              onClick={cancelEditing}
              className="text-xl font-bold text-slate-400 transition hover:text-slate-600"
              aria-label="Close edit user form"
            >
              ×
            </button>
          </div>
          <div className="grid gap-3 md:grid-cols-2">
            {(["username", "email", "company", "amr_type"] as const).map((field) => (
              <label key={field} className="flex flex-col gap-1 text-xs font-semibold uppercase tracking-wide text-slate-500">
                {field.replace("_", " ")}
                <input
                  value={editUserForm[field]}
                  onChange={(event) => setEditUserForm((prev) => ({ ...prev, [field]: event.target.value }))}
                  className="rounded-xl border border-slate-200 px-3 py-2 text-sm text-slate-800 focus:border-sky-500 focus:outline-none"
                />
              </label>
            ))}
            <label className="flex flex-col gap-1 text-xs font-semibold uppercase tracking-wide text-slate-500">
              Role
              <select
                value={editUserForm.role}
                onChange={(event) => setEditUserForm((prev) => ({ ...prev, role: event.target.value }))}
                className="rounded-xl border border-slate-200 px-3 py-2 text-sm text-slate-800 focus:border-sky-500 focus:outline-none"
              >
                <option value="user">User</option>
                <option value="admin">Admin</option>
              </select>
            </label>
            <label className="flex flex-col gap-1 text-xs font-semibold uppercase tracking-wide text-slate-500">
              Approval
              <select
                value={editUserForm.approval}
                onChange={(event) => setEditUserForm((prev) => ({ ...prev, approval: event.target.value }))}
                className="rounded-xl border border-slate-200 px-3 py-2 text-sm text-slate-800 focus:border-sky-500 focus:outline-none"
              >
                <option value="Pending">Pending</option>
                <option value="Approved">Approved</option>
                <option value="Rejected">Rejected</option>
              </select>
            </label>
          </div>
          <div className="mt-4 flex justify-end gap-2">
            <button
              type="button"
              onClick={cancelEditing}
              className="rounded-xl border border-slate-200 px-4 py-2 text-sm font-semibold text-slate-600 transition hover:bg-white"
            >
              Cancel
            </button>
            <button
              type="button"
              onClick={saveUserChanges}
              className="rounded-xl bg-[#0b74d1] px-5 py-2 text-sm font-semibold text-white shadow-lg shadow-sky-500/20"
            >
              Save Changes
            </button>
          </div>
        </div>
      ) : null}

      <div className="overflow-hidden rounded-2xl border border-slate-200 bg-white">
        <table className="w-full border-collapse text-sm">
          <thead className="bg-slate-50 text-left text-xs font-semibold uppercase tracking-wide text-slate-500">
            <tr>
              <th className="px-4 py-3">User</th>
              <th className="px-4 py-3">Email</th>
              <th className="px-4 py-3">Company</th>
              <th className="px-4 py-3">Approval</th>
              <th className="px-4 py-3 text-right">Actions</th>
            </tr>
          </thead>
          <tbody>
            {usersLoading ? (
              <tr>
                <td colSpan={5} className="px-4 py-8 text-center text-sm text-slate-500">
                  Loading users…
                </td>
              </tr>
            ) : null}
            {!usersLoading && filteredUsers.length === 0 ? (
              <tr>
                <td colSpan={5} className="px-4 py-8 text-center text-sm text-slate-500">
                  {userSearchTerm ? `No users match "${userSearchTerm}"` : "No users found."}
                </td>
              </tr>
            ) : null}
            {filteredUsers.map((user) => (
              <tr key={String(user.id)} className="border-b border-slate-100">
                <td className="px-4 py-3">
                  <div className="font-semibold text-slate-900">{user.username}</div>
                  <div className="text-xs uppercase tracking-wide text-slate-400">{user.role}</div>
                </td>
                <td className="px-4 py-3 text-slate-600">{user.email}</td>
                <td className="px-4 py-3 text-slate-600">{user.company || "—"}</td>
                <td className="px-4 py-3">
                  <span
                    className={`rounded-full px-3 py-1 text-xs font-semibold ${
                      user.approval === "Approved"
                        ? "bg-emerald-100 text-emerald-700"
                        : user.approval === "Rejected"
                        ? "bg-rose-100 text-rose-700"
                        : "bg-amber-100 text-amber-700"
                    }`}
                  >
                    {user.approval || "Pending"}
                  </span>
                </td>
                <td className="px-4 py-3">
                  <div className="flex justify-end gap-2">
                    <button
                      type="button"
                      onClick={() => startEditing(user)}
                      className="inline-flex items-center gap-1 rounded-lg border border-slate-200 px-3 py-1 text-xs font-medium text-slate-600 transition hover:border-slate-400 hover:text-slate-900"
                    >
                      <FaEdit />
                      Edit
                    </button>
                    <button
                      type="button"
                      onClick={() => {
                        setSelectedUserId(user.id);
                        handleResetUserPassword();
                      }}
                      disabled={userActionLoading && selectedUserId === user.id}
                      className="rounded-lg border border-slate-200 px-3 py-1 text-xs font-semibold text-slate-600 transition hover:border-slate-400"
                    >
                      Reset Password
                    </button>
                  </div>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      {selectedUser ? (
        <div className="rounded-2xl border border-slate-200 bg-slate-50/70 p-4">
          <div className="text-xs font-semibold uppercase tracking-[0.2em] text-slate-400">Profile</div>
          <div className="text-lg font-bold text-slate-900">{selectedUser.username}</div>
          <dl className="mt-3 grid gap-2 text-sm text-slate-600">
            <div>
              <dt className="font-semibold text-slate-700">Email</dt>
              <dd>{selectedUser.email}</dd>
            </div>
            <div>
              <dt className="font-semibold text-slate-700">Company</dt>
              <dd>{selectedUser.company || "—"}</dd>
            </div>
            <div>
              <dt className="font-semibold text-slate-700">AMR Type</dt>
              <dd>{selectedUser.amr_type || "—"}</dd>
            </div>
          </dl>
        </div>
      ) : null}
    </section>
  );
};

export default UsersPanel;

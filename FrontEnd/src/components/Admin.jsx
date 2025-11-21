import React from "react";
import "./Admin.css";

const Admin = () => {
  return (
    <div className="admin-container">
      {/* Table centered on the page */}
      <div className="table-container">
        <table className="admin-table">
          <thead>
            <tr>
              <th>Username</th>
              <th>Company</th>
              <th>MailID</th>
              <th>AMRID</th>
              <th>Approval</th>
            </tr>
          </thead>
          <tbody>
            {/* Example data */}
            <tr>
              <td>user1</td>
              <td>Company A</td>
              <td>user1@example.com</td>
              <td>AMR123</td>
              <td>Pending</td>
            </tr>
            <tr>
              <td>user2</td>
              <td>Company B</td>
              <td>user2@example.com</td>
              <td>AMR124</td>
              <td>Approved</td>
            </tr>
            <tr>
              <td>user3</td>
              <td>Company C</td>
              <td>user3@example.com</td>
              <td>AMR125</td>
              <td>Rejected</td>
            </tr>
            <tr>
              <td>user4</td>
              <td>Company D</td>
              <td>user4@example.com</td>
              <td>AMR126</td>
              <td>Pending</td>
            </tr>
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default Admin;

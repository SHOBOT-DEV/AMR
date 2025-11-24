from datetime import datetime
from backend import db
from werkzeug.security import check_password_hash


class User(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    username = db.Column(db.String(50), unique=True, nullable=False)
    email = db.Column(db.String(120), unique=True, nullable=False)
    password_hash = db.Column(db.String(200), nullable=False)
    role = db.Column(db.String(20), default="user")
    approval = db.Column(db.String(20), default="Pending")
    company = db.Column(db.String(100), nullable=True)
    amr_type = db.Column(db.String(50), nullable=True)

    def check_password(self, password):
        return check_password_hash(self.password_hash, password)

    def to_dict(self):
        return {
            "id": self.id,
            "username": self.username,
            "email": self.email,
            "role": self.role,
            "approval": self.approval,
            "company": self.company,
            "amr_type": self.amr_type,
        }


class RefreshToken(db.Model):
    """Refresh tokens extend login lifetime by rotating long-lived secrets."""

    id = db.Column(db.Integer, primary_key=True)
    token = db.Column(db.String(255), unique=True, nullable=False)
    user_id = db.Column(db.Integer, db.ForeignKey("user.id"), nullable=False)
    expires_at = db.Column(db.DateTime, nullable=False)
    revoked = db.Column(db.Boolean, default=False)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)

    user = db.relationship("User", backref=db.backref("refresh_tokens", lazy=True))

    def is_expired(self):
        return datetime.utcnow() > self.expires_at

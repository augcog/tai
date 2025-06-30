from app.config import settings
from sqladmin.authentication import AuthenticationBackend
from starlette.requests import Request


class AdminAuth(AuthenticationBackend):
    """
    Authentication backend for the admin panel.
    """

    async def login(self, request: Request) -> bool:
        form = await request.form()
        username = form.get("username")
        password = form.get("password")
        if username == settings.admin_username and password == settings.admin_password:
            # Store the username in the session to indicate authentication
            request.session.update({"admin_username": username})
            return True
        return False

    async def logout(self, request: Request) -> bool:
        request.session.clear()
        return True

    async def authenticate(self, request: Request) -> bool:
        # Check if the session contains the logged-in admin username
        return request.session.get("admin_username") == settings.admin_username

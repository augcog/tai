from playwright.sync_api import sync_playwright, TimeoutError as PlaywrightTimeoutError
import requests
import time


from .driver import Driver, Resp


class PlaywrightDriver(Driver):
    def __init__(self, headless=True, timeout=60000, max_retries=3, block_resources=True, requests_timeout=60, browser_type="chromium", skip_login_pages=True, cookies=None):
        self._play = None
        self._browser = None
        self._context = None
        self._page = None
        self._headless = headless
        self._timeout = timeout  # Default 60 seconds for Playwright
        self._max_retries = max_retries
        self._block_resources_enabled = block_resources
        self._requests_timeout = requests_timeout  # Default 60 seconds for requests
        self._browser_type = browser_type  # chromium, firefox, or webkit
        self._skip_login_pages = skip_login_pages
        self._cookies = cookies  # Optional cookies for authenticated sessions
        self._initialize_browser()

    def _initialize_browser(self):
        self._play = sync_playwright().start()

        # Browser launch args to fix HTTP/2 and bot detection issues
        launch_args = []
        if self._browser_type == "chromium":
            launch_args = [
                "--disable-http2",  # Fix ERR_HTTP2_PROTOCOL_ERROR
                "--disable-blink-features=AutomationControlled",  # Avoid bot detection
                "--disable-dev-shm-usage",
                "--no-sandbox",
            ]

        # Select browser type
        if self._browser_type == "firefox":
            browser_engine = self._play.firefox
        elif self._browser_type == "webkit":
            browser_engine = self._play.webkit
        else:  # default to chromium
            browser_engine = self._play.chromium

        # Launch browser with args
        if launch_args and self._browser_type == "chromium":
            self._browser = browser_engine.launch(headless=self._headless, args=launch_args)
        else:
            self._browser = browser_engine.launch(headless=self._headless)

        # Create context with realistic user agent
        self._context = self._browser.new_context(
            accept_downloads=True,
            user_agent="Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36"
        )

        # Add cookies if provided (for authenticated sessions)
        if self._cookies:
            self._context.add_cookies(self._cookies)

        self._page = self._context.new_page()

        # Set default timeout for all operations
        self._page.set_default_timeout(self._timeout)

        # Block unnecessary resources to speed up page loading (optional)
        if self._block_resources_enabled:
            self._page.route("**/*", self._block_resources)

    def _block_resources(self, route):
        """Block unnecessary resources to speed up page loading"""
        resource_type = route.request.resource_type
        # Block images, fonts, media, and some scripts to speed up loading
        if resource_type in ["image", "font", "media", "stylesheet"]:
            route.abort()
        else:
            route.continue_()

    def _requires_login(self, url, html_content):
        """
        Detect if a page requires login/authentication.
        Checks for common login indicators in the URL and page content.
        """
        url_lower = url.lower()
        content_lower = html_content.lower() if html_content else ""

        # Check URL patterns
        login_url_patterns = ["/login", "/signin", "/sign-in", "/auth", "/register"]
        if any(pattern in url_lower for pattern in login_url_patterns):
            return True

        # Check for common login-related keywords in content
        login_keywords = [
            "please login",
            "please sign in",
            "sign in to continue",
            "login to access",
            "authentication required",
            "you must be logged in",
            "create an account",
            "member login",
            "access denied",
            "subscription required",
        ]

        # Check if multiple keywords appear
        keyword_count = sum(1 for keyword in login_keywords if keyword in content_lower)
        if keyword_count >= 2:
            return True

        # Check for login form indicators
        login_form_indicators = [
            'type="password"',
            'name="password"',
            'id="password"',
            "login-form",
            "signin-form",
            "auth-form",
        ]
        if any(indicator in content_lower for indicator in login_form_indicators):
            # Additional check: make sure there's actual content requiring login
            if any(keyword in content_lower for keyword in ["login", "sign in", "sign-in"]):
                return True

        return False

    def download_raw(self, filename: str, url: str) -> tuple[str, Resp]:
        """
        Attempt to download the content as HTML first, and fallback to binary if an error occurs.
        Implements retry logic with exponential backoff for timeout errors.

        Parameters:
        - filename (str): Path to save the content.
        - url (str): The URL to fetch.

        Returns:
        - Resp: An object containing HTML content (if applicable), whether it is HTML, and the final URL.
        """
        # Try with retries and exponential backoff
        for attempt in range(self._max_retries):
            try:
                # Use 'domcontentloaded' for faster loading - doesn't wait for all resources
                resp = self._page.goto(
                    url,
                    wait_until="domcontentloaded",
                    timeout=self._timeout
                )

                # Check if this triggered a download (binary file)
                # If response is None or returns a non-HTML content type, it's likely a download
                if resp and resp.ok:
                    # Wait a bit for dynamic content to load
                    self._page.wait_for_timeout(2000)  # 2 seconds

                    html_content = self._page.content()

                    # Check if the page requires login
                    if self._skip_login_pages and self._requires_login(resp.url, html_content):
                        print(f"⚠️  Skipping login-protected page: {url}")
                        raise Exception(f"LOGIN_REQUIRED: {url} requires authentication")

                    filename = self._page.title().strip() or filename.rstrip('.html') + '.html'

                    with open(filename, "w", encoding="utf-8") as f:
                        f.write(html_content)

                    return filename, Resp(
                        html_content=html_content,
                        is_html=True,
                        true_url=resp.url,
                    )
                else:
                    # Navigation failed, likely a download - fall back to requests
                    raise Exception("Navigation returned unsuccessful response")

            except PlaywrightTimeoutError as e:
                if attempt < self._max_retries - 1:
                    # Exponential backoff: wait 2^attempt seconds
                    wait_time = 2 ** attempt
                    print(f"Timeout on attempt {attempt + 1}/{self._max_retries}, retrying in {wait_time}s...")
                    time.sleep(wait_time)
                else:
                    # Final attempt failed, raise the error
                    raise Exception(f"Failed to load {url} after {self._max_retries} attempts: {e}")

            except Exception as e:
                # For non-HTML content or other errors, try downloading as binary
                print(f"Playwright failed ({e}), trying direct download...")
                try:
                    response = requests.get(url, stream=True, timeout=self._requests_timeout)
                    response.raise_for_status()

                    with open(filename, "wb") as file:
                        for chunk in response.iter_content(chunk_size=8192):
                            file.write(chunk)

                    return filename, Resp(
                        html_content=None,
                        is_html=False,
                        true_url=response.url,
                    )
                except Exception as download_error:
                    if attempt < self._max_retries - 1:
                        wait_time = 2 ** attempt
                        print(f"Download failed on attempt {attempt + 1}/{self._max_retries}, retrying in {wait_time}s...")
                        time.sleep(wait_time)
                    else:
                        raise Exception(f"Failed to download {url} after {self._max_retries} attempts: {download_error}")

    def close(self):
        if self._page:
            self._page.close()
        if self._context:
            self._context.close()
        if self._browser:
            self._browser.close()
        if self._play:
            self._play.stop()

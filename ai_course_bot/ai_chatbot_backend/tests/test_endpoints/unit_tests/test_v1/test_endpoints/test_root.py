def test_root_redirect_no_follow(client_unit):
    """
    Test the immediate redirect response without following.
    Expect 302 or 307 with a Location header set to '/docs'.
    """
    response = client_unit.get("/", allow_redirects=False)
    assert response.status_code in (302, 307)
    assert response.headers["Location"] == "/docs"


def test_root_redirect_with_follow(client_unit):
    """
    Test that following the redirect lands on the /docs endpoint.
    """
    response = client_unit.get("/")  # allow_redirects=True by default
    assert response.status_code == 200
    assert str(response.url).endswith("/docs")

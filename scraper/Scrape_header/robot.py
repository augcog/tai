import urllib.robotparser as robotparser


def get_crawl_delay(site_url, user_agent="*"):
    robots_url = site_url.rstrip('/') + '/robots.txt'
    
    rp = robotparser.RobotFileParser()
    rp.set_url(robots_url)
    try:
        rp.read()
        return rp.crawl_delay(user_agent)
    except:
        print("Error accessing or parsing robots.txt.")
        return None

# Example usage:
delay_time = get_crawl_delay('https://wiki.ros.org/')
if delay_time:
    print(f"Crawl-delay is set to {delay_time} seconds.")
else:
    print("No Crawl-delay specified or unable to fetch robots.txt.")

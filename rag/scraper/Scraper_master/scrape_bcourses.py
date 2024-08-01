from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time
import subprocess
import pychrome
import requests

options = webdriver.ChromeOptions()
options.add_argument("--start-maximized")
options.add_argument("--enable-logging")
options.add_argument("--remote-debugging-port=9222")  # Enable remote debugging
driver = webdriver.Chrome(options=options)

driver.get('https://bcourses.berkeley.edu/courses/1533392/external_tools/90481')
wait = WebDriverWait(driver, 10)

calnet_id = ''
passphrase = ''
calnet_id_field = wait.until(EC.presence_of_element_located((By.ID, 'username')))  
passphrase_field = wait.until(EC.presence_of_element_located((By.ID, 'password')))

calnet_id_field.send_keys(calnet_id)
passphrase_field.send_keys(passphrase)

sign_in_button = wait.until(EC.element_to_be_clickable((By.ID, 'submit')))
sign_in_button.click()

wait.until(EC.url_to_be('https://bcourses.berkeley.edu/courses/1533392/external_tools/90481'))
wait = WebDriverWait(driver, 30)

while True:
    try:
        iframe = wait.until(EC.presence_of_element_located((By.CSS_SELECTOR, "iframe.tool_launch")))
        driver.switch_to.frame(iframe)
        
        load_more_button = wait.until(EC.element_to_be_clickable((By.XPATH, "//a[contains(text(), 'Load More')]")))
        load_more_button.click()
        print("Clicked 'Load More' button.")
        
        time.sleep(5)
        driver.switch_to.default_content()
        
    except Exception as e:
        #print(f"Exception occurred: {e}")
        break

print("Complete")

time.sleep(2)

# Switch to iframe to extract links
#iframe = wait.until(EC.presence_of_element_located((By.CSS_SELECTOR, "iframe.tool_launch")))
#driver.switch_to.frame(iframe)

# Find all <li> elements within the <ul> with class 'thumbnails channel-entries-gallery isotope'
ul_element = driver.find_element(By.ID, 'gallery')
li_elements = ul_element.find_elements(By.CLASS_NAME, 'galleryItem')

# Open each link in a new tab and process it
for li in li_elements:
    try:
        link = li.find_element(By.TAG_NAME, 'a').get_attribute('href')
        driver.execute_script("window.open(arguments[0], '_blank');", link)
        
        # Switch to the new tab
        driver.switch_to.window(driver.window_handles[-1])
        
        # Wait for the page to load
        time.sleep(5)
        
        iframe = wait.until(EC.presence_of_element_located((By.CSS_SELECTOR, "iframe.mwEmbedKalturaIframe#kplayer_ifp")))
        driver.switch_to.frame(iframe)

        # Click the play button
        play_button = wait.until(EC.element_to_be_clickable((By.CLASS_NAME, "icon-play  comp largePlayBtn  largePlayBtnBorder")))
        play_button.click()
        "clicked"

        break
        
        # Wait for the network traffic to capture
        time.sleep(10)
        
        # Setup Chrome DevTools Protocol
        debugger_url = "http://127.0.0.1:9222/json"  # Default remote debugging port
        m3u8_url = get_m3u8_url(debugger_url)
        
        if m3u8_url:
            # Download video
            output_file = f"video_{int(time.time())}.mp4"
            download_video(m3u8_url, output_file)
            print(f"Downloaded video to {output_file}")
        
        # Close the tab
        driver.close()
        driver.switch_to.window(driver.window_handles[0])
        
        time.sleep(2)  # Wait a bit before opening the next tab
        break
        
    except Exception as e:
        print(f"Exception while processing <li>: {e}")

# Close the browser
driver.quit()

def get_m3u8_url(debugger_url):
    # Connect to Chrome DevTools Protocol
    response = requests.get(debugger_url)
    tabs = response.json()
    if not tabs:
        return None
    
    tab_id = tabs[0]['id']
    ws_url = tabs[0]['webSocketDebuggerUrl']
    
    # Connect to WebSocket
    browser = pychrome.Browser(url=ws_url)
    tab = browser.list_tab()[0]
    tab.start()

    # Enable network domain
    tab.call_method('Network.enable')

    m3u8_url = None

    # Event handler for network responses
    def response_received(**kwargs):
        nonlocal m3u8_url
        url = kwargs.get('response', {}).get('url', '')
        if '.m3u8' in url:
            m3u8_url = url

    tab.set_listener('Network.responseReceived', response_received)

    # Wait for network responses
    time.sleep(10)

    tab.stop()
    return m3u8_url

def download_video(m3u8_url, output_file):
    command = [
        'ffmpeg',
        '-protocol_whitelist', 'file,http,https,tcp,tls,crypto',
        '-i', m3u8_url,
        '-c', 'copy',
        output_file
    ]
    subprocess.run(command)
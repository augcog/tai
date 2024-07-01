from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time

options = webdriver.ChromeOptions()
options.add_argument("--start-maximized")
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
wait = WebDriverWait(driver, 10)

load_more_button = wait.until(EC.element_to_be_clickable((By.XPATH, '//a[contains(@href, "endlessScrollersPrototype.loadNextPage") and contains(text(), "Load More")]')))
load_more_button.click()

#load_more_button = wait.until(EC.element_to_be_clickable((By.LINK_TEXT, 'Load More')))
#load_more_button.click()


#while True:
    #try:
        #load_more_button = wait.until(EC.element_to_be_clickable((By.LINK_TEXT, 'Load More')))
        #load_more_button.click()
        #time.sleep(5)  # Wait for content to load
    #except:
        #break
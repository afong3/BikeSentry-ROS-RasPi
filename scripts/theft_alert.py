import requests


def main():
    print("hello")
    url = r"https://bike-sentry-api-2vgam74tba-uc.a.run.app/theft_alert/T0"
    res = requests.post(url)
    print(res)



if __name__ == "__main__":
    main()

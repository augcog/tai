import openai
import json

openai.api_key = "empty"
openai.api_base = "http://localhost:8000/v1"

str = "a"
count = 1000
token= 0
while(token < 300):
    print(count)
    str=count*"a"
    b=openai.Embedding.create(model="text-embedding-ada-002", input=str)
    # print(b)
    # print(json.dumps(b, indent=2))
    count+=100
    token=b['usage']['total_tokens']
    print(f"token: {token}")

def chat_completion(system_message, human_message):
    messages=human_message+"\n---\n"+system_message
    # if model=='local':
    #     prompt=wizard_coder(history)
    # elif model=='openai':
    #     prompt=gpt(history)
    # print(prompt)
    # completion = openai.ChatCompletion.create(
    #     model='gpt-3.5-turbo', messages=messages, temperature=0,
    # )

    completion = openai.Completion.create(model='gpt-3.5-turbo', prompt=messages, temperature=0, max_tokens=500)
    print(completion)
    # print(completion)

    # answer=completion['choices'][0]['message']["content"]
    answer=completion['choices'][0]['text']
    # print(answer)
#
    return answer

# prompt = """
#     The 1899 Kentucky gubernatorial election was held on November 7, 1899, to choose the 33rd governor of Kentucky. The incumbent, Republican William O'Connell Bradley, was term-limited and unable to seek re-election.
#
# During a contentious and chaotic convention at the Music Hall in Louisville, the Democratic Party nominated state Senator William Goebel. A dissident faction of the party, styling themselves the "Honest Election Democrats", were angered by Goebel's political tactics at the Music Hall convention and later held their own convention. They nominated former Governor John Y. Brown. Republicans nominated state Attorney General William S. Taylor, although Governor Bradley favored another candidate and lent Taylor little support in the ensuing campaign. In the general election, Taylor won by a vote of 193,714 to 191,331. Brown garnered 12,040 votes, more than the difference between Taylor and Goebel. The election results were challenged on grounds of voter fraud, but the state Board of Elections, created by a law Goebel had sponsored and stocked with pro-Goebel commissioners, certified Taylor's victory.
#
# An incensed Democratic majority in the Kentucky General Assembly created a committee to investigate the charges of voter fraud, even as armed citizens from heavily Republican eastern Kentucky poured into the state capital under the auspices of keeping Democrats from stealing the election. Before the investigative committee could report, Goebel was shot by an unknown assassin while entering the state capitol on January 30, 1900. As Goebel lay in a nearby hotel being treated for his wounds, the committee issued its report recommending that the General Assembly invalidate enough votes to give the election to Goebel. The report was accepted, Taylor was removed from office, and Goebel was sworn into office on January 31. He died three days later on February 3.
#
# Lieutenant Governor J. C. W. Beckham ascended to the office of governor, and he and Taylor waged a protracted court battle over the governorship. Beckham won the case on appeal, and Taylor fled to Indiana to escape prosecution as an accomplice in Goebel's murder. A total of sixteen people were charged in connection with the assassination. Five went to trial; two of those were acquitted. Each of the remaining three were convicted in trials fraught with irregularities and were eventually pardoned by subsequent governors. The identity of Goebel's assassin remains a mystery.
#
# Background
# In the 1895 gubernatorial election, Kentucky elected its first-ever Republican governor, William O. Bradley. Bradley was able to capitalize both on divisions within the Democratic Party over the Free Silver issue and on the presence of a strong third-party candidate, Populist Thomas S. Pettit, to secure victory in the general election by just under 9,000 votes. This election marked the beginning of nearly thirty years of true two-party competition in Kentucky politics, but the state Constitution barred the governor from being re-elected.[1]
#
# A powerful Democratic foe of Bradley had begun his rise to power in the Kentucky Senate. Kenton County's William Goebel became the leader of a new group of young Democrats who were seen as enemies of large corporations, particularly the Louisville and Nashville Railroad, and friends of the working man. Goebel was known as aloof and calculating. Unmarried and with few close friends of either gender, he was singularly driven by political power.[2]
#
# Goebel was chosen president pro tem of the Senate for the 1898 legislative session. On February 1, 1898, he sponsored a measure later called the Goebel Election Law.[3] The law created a Board of Election Commissioners, appointed by the General Assembly, who were responsible for choosing election commissioners in all of Kentucky's counties and were empowered to decide disputed elections.[3] Because the General Assembly was heavily Democratic, the law was attacked as blatantly partisan and self-serving to Goebel; it was opposed even by some Democrats.[4] Nevertheless, Goebel was able to hold enough members of his party together to override Governor Bradley's veto, making the bill law.[4] As leader of the legislative majority, Goebel essentially hand-picked the members of the Election Commission.[5] He chose three staunch Democrats—W. S. Pryor, former chief justice of the Kentucky Court of Appeals; W. T. Ellis, former U. S. Representative from Daviess County; and C. B. Poyntz, former head of the state railroad commission.[5] Republicans organized a test case against the law, but the Court of Appeals found it constitutional.
# Three Democratic candidates had announced intentions to run for governor in 1899—Goebel, former Kentucky Attorney General P. Wat Hardin, and former congressman William J. Stone.[7] Hardin, a native of Mercer County, had the backing of the Louisville and Nashville Railroad.[7] Lyon County's Stone had the backing of the state's agricultural interests.[7] Goebel generally had the backing of urban voters.[7] Going into the party's nominating convention, Hardin was the favorite to win the nomination.[7] Knowing that combining forces was the only way to prevent Hardin's nomination, representatives of Goebel and Stone met on June 19, 1899, to work out a deal.[8] According to Urey Woodson, a Goebel representative at the meeting, the two sides signed an agreement whereby half of the Louisville delegation, which was committed to Goebel, would vote for Stone.[7] Both men agreed that, should one of them be defeated or withdraw from the race, they would encourage their delegates to vote for the other rather than support Hardin.[7]
#
# The Democratic nominating convention began on June 20, at the Music Hall on Market Street in Louisville.[7] The first order of business was to nominate a convention chairman. Ollie M. James, a supporter of Stone, nominated Judge David Redwine.[8] When Woodson seconded the nomination, the deal between Stone and Goebel became apparent to all.[8] Hardin supporters nominated William H. Sweeney, but the Stone-Goebel alliance elected Redwine.[9] The membership of several county delegations was challenged; these cases would be decided by the credentials committee.[9] This committee was also stacked against Hardin; his supporters made up just four of the thirteen members.[9] Prolonged deliberations by the credentials committee caused the delegates to become restless, and hundreds of people—both delegates and non-delegates—entered the Music Hall attempting to disrupt the convention.[7] When Redwine summoned Louisville city police to the hall to maintain order, Hardin supporters accused him of using intimidation tactics.[10] The credentials committee finally issued its report on June 23.[10] Of the twenty-eight cases where delegates were contested, twenty-six of them were decided in favor of Goebel or Stone supporters.[10]
#
# Formal nominations began the following day.[10] Hardin felt as though he had been cheated and withdrew his candidacy, although some loyal delegates continued to vote for him.[11] Delegate John Stockdale Rhea nominated Stone.[10] Stone believed his agreement with Goebel meant, with Hardin's withdrawal, Goebel would instruct his delegates to vote for Stone, maintaining a unified party.[10] That understanding vanished when another delegate nominated Goebel.[10] Stone was further incensed when all of the Louisville delegation voted for Goebel instead of being split between Stone and Goebel, as the two men had previously agreed.[10] In retaliation, some Stone supporters began to back Hardin.[10] Seeing the breakdown of the Stone-Goebel alliance, Hardin reversed his withdrawal.[12] After numerous ballots, the convention was deadlocked on the night of June 24 with each candidate receiving about one-third of the votes.[13] No deliberations were held on Sunday, June 25, and when the delegates reconvened on Monday, June 26, the hall was filled with police per Redwine's request.[10] Rhea requested that the police be removed to prevent intimidation, but Redwine ruled the motion out of order.[10] Another delegate appealed Redwine's decision, and, in violation of parliamentary rule, Redwine ruled the appeal out of order.[12] Angered by Redwine's obviously biased rulings, delegates for Stone and Hardin then began trying to disrupt the convention by blowing horns, singing, yelling, and standing on chairs.[10] Although voting was attempted, many delegates abstained because they were unable to hear and understand what was going on.[10] When the voting—such as it was—ended, the chair announced that Goebel had a majority of the votes cast, but Goebel sent word to Redwine that he would accept the nomination only if he received an absolute majority of the delegates.[14] Further attempts to vote were likewise disrupted, and the meeting adjourned for the day.[10]
#
# On the morning of June 27, the hall was orderly.[10] Stone and Hardin both called for the convention to adjourn sine die.
#     """
# print(chat_completion("What's this document about?", prompt))
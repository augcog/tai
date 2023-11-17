import os
import pickle
import openai
openai.api_key = "empty"
openai.api_base = "http://localhost:8000/v1"
def chat_completion(system_message, human_message):
    history = [{"role": "system", "content": system_message}, {"role": "user", "content": human_message}]
    # if model=='local':
    #     prompt=wizard_coder(history)
    # elif model=='openai':
    #     prompt=gpt(history)
    # print(prompt)
    # completion = openai.ChatCompletion.create(
    #     model='gpt-3.5-turbo', messages=messages, temperature=0,
    # )
    # prompt=wizard_coder(history)
    completion=openai.ChatCompletion.create(model='gpt-3.5-turbo',messages=history, temperature=0, max_tokens=500)
    # completion = openai.Completion.create(model='gpt-3.5-turbo', prompt=prompt, temperature=0, max_tokens=500)
    # print(completion)
    # print(completion)

    # answer=completion['choices'][0]['message']["content"]
    answer=completion['choices'][0]['message']["content"]
    return answer

question="How are the forces applied by inelastic tendons in a robotic finger related to the torques generated at the finger's joints?"
doc="""
### Inelastic tendons

Consider a finger which is actuated by a set of inelastic tendons, such as the one shown in Figure 6.7. Each tendon consists of an inextensible cable connected to a force generator, such as a DC motor. For simplicity, we assume that each tendon/actuator pair is connected either between the base of the hand and a link of the finger, or between two links of the finger. Interconnections between tendons are not allowed. We wish to describe how forces applied at the end of the tendons are related to the torques applied at the joints.

Note that even though each tendon can be connected to only one link, pulling on a tendon may generate forces on many joints. This occurs because as we pull on a tendon, it exerts forces all along its length against whatever parts of the mechanism are holding it in place. This coupling is difficult to eliminate without awkward routing of the tendons.

We model the routing of each tendon by an _extension function_, \(h_{i}:Q\to\mathbb{R}\). The extension function measures the displacement of the end of the tendon as a function of the joint angles of the finger. For simple tendon networks composed of pulleys, such as those shown in Figure 6.7, the tendon extension is a linear function of the joint angles

\[h_{i}(\theta)=l_{i}\pm r_{i1}\theta_{1}\pm\dots\pm r_{in}\theta_{n},\]

where \(l_{i}\) is the nominal extension (at \(\theta=0\)) and \(r_{ij}\) is the radius of the pulley at the jth joint. The sign depends on whether the tendon path gets longer or shorter when the angle is changed in a positive sense.

More complicated tendon geometries may involve nonlinear functions of the joint angles. For example, for the joint pictured in Figure 6.8, the top tendon has an extension function of the form

\[h_{1}(\theta)=l_{1}+2\sqrt{a^{2}+b^{2}}\cos\left(\tan^{-1}\left(\frac{a}{b} \right)+\frac{\theta}{2}\right)-2b\qquad\theta>0,\]
"""

print("Answer with doc: " + chat_completion("Answer the question based on the document. Make the answer short",question+"\n---\n"+doc))
print("Answer without doc: " + chat_completion("Answer the question. Make the answer short",question))
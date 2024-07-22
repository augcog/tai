import { UseChatHelpers } from 'ai/react'

import { Button } from '@/tai/components/ui/button'
import { ExternalLink } from '@/tai/components/external-link'
import { IconArrowRight } from '@/tai/components/ui/icons'
import Image from 'next/image'

const exampleMessages = [
  {
    heading: 'Explain course grading system',
    message: `What is grading like in this course?`
  },
  {
    heading: 'Summarize course content',
    message: 'Summarize what will I learn in this course \n'
  }
]

export function EmptyScreen({ setInput }: Pick<UseChatHelpers, 'setInput'>) {
  return (
    <div className="mx-auto max-w-2xl px-4">
      <div className="rounded-lg border bg-background p-8">
        <div className="flex justify-center mb-4">
          <Image src="/TAI_prompt.png" alt="logo" width={100} height={100} />
        </div>
        <h1 className="mb-2 text-lg font-semibold">
          Welcome to Teaching Assistant Intelligence
        </h1>
        <p className="h-auto p-0 text-base">
          • You may chat with our public GPT server while remaining anonymous.
        </p>
        <br />

        <p className="h-auto p-0 text-base">
          • For Berkeley users with valid campus accounts, you may authenticate
          and then chat with private GPT servers that cover teaching assistance
          for all supported Berkeley courses.
        </p>
        <br />

        <p className="h-auto p-0 text-base">
          • Your questions and private information will never be shared to any
          third-party businesses.
        </p>
        {/* <div className="mt-4 flex flex-col items-start space-y-2">
          {exampleMessages.map((message, index) => (
            <Button
              key={index}
              variant="link"
              className="h-auto p-0 text-base"
              onClick={() => setInput(message.message)}
            >
              <IconArrowRight className="mr-2 text-muted-foreground" />
              {message.heading}
            </Button>
          ))}
        </div> */}
      </div>
    </div>
  )
}
